/*
 * Copyright (c) [2026] Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free,
 * non-exclusive license under copyrights and patents it now or hereafter
 * owns or controls to make, have made, use, import, offer to sell and sell
 * ("Utilize") this software subject to the terms herein.  With respect to
 * the foregoing patent license, such license is granted  solely to the extent
 * that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include
 * this software, other than combinations with devices manufactured by or
 * for TI (“TI Devices”).  No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce
 * this license (including the above copyright notice and the disclaimer
 * and (if applicable) source code license limitations below) in the
 * documentation and/or other materials provided with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted
 * provided that the following conditions are met:
 *
 * *	No reverse engineering, decompilation, or disassembly of this software
 *      is permitted with respect to any software provided in binary form.
 *
 * *	Any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *	Nothing shall obligate TI to provide you with source code for the
 *      software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution
 * of the source code are permitted provided that the following conditions are met:
 *
 * *	Any redistribution and use of the source code, including any resulting
 *      derivative works, are licensed by TI for use only with TI Devices.
 *
 * *	Any redistribution and use of any object code compiled from the source
 *      code and any resulting derivative works, are licensed by TI for use
 *      only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its
 * suppliers may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EV
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gsttitvm.h"
#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <json-c/json.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <algorithm>

/* TVM C++ Runtime includes */
#include <tvm/runtime/module.h>
#include <tvm/runtime/registry.h>
#include <tvm/runtime/packed_func.h>
#include <tvm/runtime/ndarray.h>
#include <tvm/runtime/container/map.h>

extern "C"
{
#include "rpmsg.h"
#include "dmabuf.h"
}

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>

using namespace
    tvm::runtime;

/*
 * tvm-model-daemon protocol (mirrors include/tvm_daemon_proto.h from the
 * rpmsg-dma-pd edge-ai example app). The DSP compute channel only supports
 * one client at a time; on boards running tvm-model-daemon.service, that
 * daemon already owns the c7x_compute session, so titvm must reuse it
 * over this socket rather than loading the model in-process.
 */
#define TVM_DAEMON_SOCKET_PATH "/var/run/tvm-inference.sock"
#define TVM_DAEMON_MAGIC       0x544D5644u      /* 'TMVD' */

enum TvmDaemonMsgType
{
  TVM_DAEMON_MSG_PING = 0,
  TVM_DAEMON_MSG_PONG = 1,
  TVM_DAEMON_MSG_INFER_REQ = 2,
  TVM_DAEMON_MSG_INFER_RESP = 3,
  TVM_DAEMON_MSG_ERROR_RESP = 4,
};

struct TvmDaemonHeader
{
  uint32_t
      magic;
  uint32_t
      type;
  uint32_t
      len;
} __attribute__((packed));

/* RPMsg message types for audio mode */
#define C7X_MSG_DEINTERLEAVE      0x1040        // Same message type for both operations
#define C7X_MSG_DEINTERLEAVE_RESP 0x2040
#define C7X_MSG_INTERLEAVE        0x1040        // flag parameter differentiates
#define C7X_MSG_INTERLEAVE_RESP   0x2040
#define C7X_STATUS_SUCCESS        0

struct c7x_msg_hdr
{
  uint32_t
      type;
  uint32_t
      seq;
  uint32_t
      len;
  int32_t
      status;
} __attribute__((packed));

struct audio_msg
{
  struct c7x_msg_hdr
      hdr;
  uint32_t
      input_buffer;
  uint32_t
      output_buffer;
  uint32_t
      input_frame;              // Number of frames (401 for GCRN)
  uint32_t
      fft_size;                 // FFT size (320 for GCRN)
  uint32_t
      flag;                     // 0=deinterleave, 1=interleave
} __attribute__((packed));

GST_DEBUG_CATEGORY_STATIC (gst_ti_tvm_debug_category);
#define GST_CAT_DEFAULT gst_ti_tvm_debug_category

/* Prototypes */
static void
gst_ti_tvm_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec);

static void
gst_ti_tvm_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec);

static void
gst_ti_tvm_dispose (GObject * object);

static void
gst_ti_tvm_finalize (GObject * object);

static
    gboolean
gst_ti_tvm_start (GstBaseTransform * trans);

static
    gboolean
gst_ti_tvm_stop (GstBaseTransform * trans);

static GstCaps *
gst_ti_tvm_transform_caps (GstBaseTransform * trans,
    GstPadDirection direction, GstCaps * caps, GstCaps * filter);

static
    GstFlowReturn
gst_ti_tvm_prepare_output_buffer (GstBaseTransform * trans,
    GstBuffer * inbuf, GstBuffer ** outbuf);

static
    GstFlowReturn
gst_ti_tvm_transform (GstBaseTransform * trans,
    GstBuffer * inbuf, GstBuffer * outbuf);

/* TVM-specific functions */
static
    gboolean
gst_ti_tvm_load_model (GstTiTvm * tvm);

static
    gint
gst_ti_tvm_daemon_connect (GstTiTvm * tvm);

static
    GstFlowReturn
gst_ti_tvm_run_inference (GstTiTvm * tvm,
    gfloat * input_data, gsize input_size);

static
    GstFlowReturn
gst_ti_tvm_run_inference_daemon (GstTiTvm * tvm,
    gfloat * input_data, gsize input_size);

static gchar *
gst_ti_tvm_load_json_file (const gchar * file_path);

static gchar *
gst_ti_tvm_load_param_file (const gchar * file_path, gsize * file_size);

static
    gboolean
gst_ti_tvm_parse_shape_from_json (GstTiTvm * tvm, const gchar * graph_json,
    std::vector < int64_t > &input_shape, std::vector < int64_t > &output_shape,
    gchar ** input_name);

static void
gst_ti_tvm_load_class_map (GstTiTvm * tvm);

static void
gst_ti_tvm_print_top_predictions (GstTiTvm * tvm, GstBuffer * inbuf,
    const gfloat * scores, gsize count);

/* Pad templates */
static
    GstStaticPadTemplate
    sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("application/octet-stream")
    );

static
    GstStaticPadTemplate
    src_factory = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("application/octet-stream")
    );


#define gst_ti_tvm_parent_class parent_class
G_DEFINE_TYPE_WITH_CODE (GstTiTvm, gst_ti_tvm, GST_TYPE_BASE_TRANSFORM,
    GST_DEBUG_CATEGORY_INIT (gst_ti_tvm_debug_category, "titvm", 0,
        "TI TVM inference element"));

static void
gst_ti_tvm_class_init (GstTiTvmClass * klass)
{
  GObjectClass *
      gobject_class = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *
      base_transform_class = GST_BASE_TRANSFORM_CLASS (klass);

  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_static_pad_template_get (&sink_factory));

  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_static_pad_template_get (&src_factory));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "TI TVM Inference", "Transform/ML",
      "TVM inference on TI processors with C7x DSP acceleration",
      "Pratham Deshmukh <p-deshmukh@ti.com>");

  gobject_class->set_property = gst_ti_tvm_set_property;
  gobject_class->get_property = gst_ti_tvm_get_property;
  gobject_class->dispose = gst_ti_tvm_dispose;
  gobject_class->finalize = gst_ti_tvm_finalize;

  base_transform_class->start = GST_DEBUG_FUNCPTR (gst_ti_tvm_start);
  base_transform_class->stop = GST_DEBUG_FUNCPTR (gst_ti_tvm_stop);
  base_transform_class->transform = GST_DEBUG_FUNCPTR (gst_ti_tvm_transform);
  base_transform_class->transform_caps =
      GST_DEBUG_FUNCPTR (gst_ti_tvm_transform_caps);
  base_transform_class->prepare_output_buffer =
      GST_DEBUG_FUNCPTR (gst_ti_tvm_prepare_output_buffer);

  base_transform_class->passthrough_on_same_caps = FALSE;

  /* Properties */
  g_object_class_install_property (gobject_class, PROP_MODEL_PATH,
      g_param_spec_string ("model-path", "Model Path",
          "Path to TVM artifacts directory", DEFAULT_MODEL_PATH,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_CLASS_MAP_PATH,
      g_param_spec_string ("class-map-path", "Class Map Path",
          "Optional: YAML file of ordered class names (e.g. yamnet_class_map.yml) "
          "for live top-k prediction printing. Empty (default) disables printing.",
          DEFAULT_CLASS_MAP_PATH,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_TOP_K,
      g_param_spec_uint ("top-k", "Top K",
          "Number of top predictions to print per window when class-map-path is set",
          1, 521, DEFAULT_TOP_K,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}

static void
gst_ti_tvm_init (GstTiTvm * tvm)
{
  tvm->model_path = g_strdup (DEFAULT_MODEL_PATH);
  tvm->class_map_path = g_strdup (DEFAULT_CLASS_MAP_PATH);
  tvm->top_k = DEFAULT_TOP_K;

  tvm->class_names = NULL;
  tvm->num_class_names = 0;
  tvm->window_counter = 0;

  tvm->tvm_initialized = FALSE;
  tvm->graph_executor = NULL;
  tvm->set_input_func = NULL;
  tvm->run_func = NULL;
  tvm->get_output_func = NULL;

  tvm->auto_input_name = NULL;
  tvm->auto_input_shape = new std::vector < int64_t > ();
  tvm->auto_output_shape = new std::vector < int64_t > ();

  tvm->final_output = NULL;
  tvm->output_num_floats = 0;

  tvm->daemon_fd = -1;
  tvm->daemon_output_buf = NULL;
  tvm->daemon_output_buf_size = 0;

  memset (&tvm->perf_data, 0, sizeof (tvm->perf_data));

  tvm->inference_completed = FALSE;
}

static void
gst_ti_tvm_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTiTvm *
      tvm = GST_TI_TVM (object);

  GST_OBJECT_LOCK (tvm);
  switch (property_id) {
    case PROP_MODEL_PATH:
      g_free (tvm->model_path);
      tvm->model_path = g_value_dup_string (value);
      break;
    case PROP_CLASS_MAP_PATH:
      g_free (tvm->class_map_path);
      tvm->class_map_path = g_value_dup_string (value);
      break;
    case PROP_TOP_K:
      tvm->top_k = g_value_get_uint (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (tvm);
}

static void
gst_ti_tvm_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstTiTvm *
      tvm = GST_TI_TVM (object);

  GST_OBJECT_LOCK (tvm);
  switch (property_id) {
    case PROP_MODEL_PATH:
      g_value_set_string (value, tvm->model_path);
      break;
    case PROP_CLASS_MAP_PATH:
      g_value_set_string (value, tvm->class_map_path);
      break;
    case PROP_TOP_K:
      g_value_set_uint (value, tvm->top_k);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (tvm);
}

static void
gst_ti_tvm_dispose (GObject * object)
{
  /* Clean up any remaining resources */

  G_OBJECT_CLASS (parent_class)->dispose (object);
}

static void
gst_ti_tvm_finalize (GObject * object)
{
  GstTiTvm *
      tvm = GST_TI_TVM (object);

  g_free (tvm->model_path);
  g_free (tvm->class_map_path);
  g_free (tvm->auto_input_name);

  if (tvm->class_names) {
    guint i;
    for (i = 0; i < tvm->num_class_names; i++)
      g_free (tvm->class_names[i]);
    g_free (tvm->class_names);
    tvm->class_names = NULL;
  }

  if (tvm->auto_input_shape) {
    delete static_cast < std::vector < int64_t > *>(tvm->auto_input_shape);
    tvm->auto_input_shape = NULL;
  }
  if (tvm->auto_output_shape) {
    delete static_cast < std::vector < int64_t > *>(tvm->auto_output_shape);
    tvm->auto_output_shape = NULL;
  }

  if (tvm->perf_data.inference_times) {
    g_free (tvm->perf_data.inference_times);
    tvm->perf_data.inference_times = NULL;
  }

  g_free (tvm->daemon_output_buf);
  tvm->daemon_output_buf = NULL;

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static gboolean
gst_ti_tvm_start (GstBaseTransform * trans)
{
  GstTiTvm *tvm = GST_TI_TVM (trans);

  GST_DEBUG_OBJECT (tvm, "Starting TI TVM inference element");
  g_print ("TI TVM Inference Element\n");
  g_print ("========================\n");

  /* Initialize TVM runtime */
  if (!gst_ti_tvm_load_model (tvm)) {
    GST_ERROR_OBJECT (tvm, "Failed to load TVM model");
    return FALSE;
  }

  /* Optional: live top-k class prediction printing */
  gst_ti_tvm_load_class_map (tvm);

  tvm->window_counter = 0;

  /* Performance tracking (single run only) */
  tvm->perf_data.inference_times = g_new0 (gint64, 1);

  GST_DEBUG_OBJECT (tvm, "TI TVM element started successfully");

  return TRUE;
}

static gboolean
gst_ti_tvm_stop (GstBaseTransform * trans)
{
  GstTiTvm *tvm = GST_TI_TVM (trans);

  GST_DEBUG_OBJECT (tvm, "Stopping TI TVM inference element");

  tvm->tvm_initialized = FALSE;

  if (tvm->daemon_fd >= 0) {
    close (tvm->daemon_fd);
    tvm->daemon_fd = -1;
  }

  if (tvm->graph_executor) {
    delete (Module *) tvm->graph_executor;
    tvm->graph_executor = NULL;
  }
  if (tvm->set_input_func) {
    delete (PackedFunc *) tvm->set_input_func;
    tvm->set_input_func = NULL;
  }
  if (tvm->run_func) {
    delete (PackedFunc *) tvm->run_func;
    tvm->run_func = NULL;
  }
  if (tvm->get_output_func) {
    delete (PackedFunc *) tvm->get_output_func;
    tvm->get_output_func = NULL;
  }
  if (tvm->final_output) {
    delete (NDArray *) tvm->final_output;
    tvm->final_output = NULL;
  }

  GST_DEBUG_OBJECT (tvm, "TI TVM element stopped");

  return TRUE;
}

/* Caps negotiation */
static GstCaps *
gst_ti_tvm_transform_caps (GstBaseTransform * trans, GstPadDirection direction,
    GstCaps * caps, GstCaps * filter)
{
  GstCaps *othercaps;

  othercaps = gst_caps_new_simple ("application/octet-stream", NULL, NULL);

  if (filter) {
    GstCaps *intersect = gst_caps_intersect_full (othercaps, filter,
        GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (othercaps);
    othercaps = intersect;
  }

  return othercaps;
}

/* Prepare output buffer with correct size */
static GstFlowReturn
gst_ti_tvm_prepare_output_buffer (GstBaseTransform * trans, GstBuffer * inbuf,
    GstBuffer ** outbuf)
{
  GstTiTvm *tvm = GST_TI_TVM (trans);
  gsize output_size;

  /* If we already know the output size from previous inference, use it */
  if (tvm->output_num_floats > 0) {
    output_size = tvm->output_num_floats * sizeof (float);
    GST_DEBUG_OBJECT (tvm, "Allocating output buffer: %zu bytes (known size)",
        output_size);
  } else {
    /* First run:
     * Allocate buffer same size as input.
     * After first inference, we'll know exact size for subsequent buffers. */
    output_size = gst_buffer_get_size (inbuf);
    GST_DEBUG_OBJECT (tvm,
        "Allocating output buffer: %zu bytes (input size, first run)",
        output_size);
  }

  /* Allocate output buffer with correct size */
  *outbuf = gst_buffer_new_allocate (NULL, output_size, NULL);
  if (!*outbuf) {
    GST_ERROR_OBJECT (tvm, "Failed to allocate output buffer of size %zu",
        output_size);
    return GST_FLOW_ERROR;
  }

  /* This is a freshly allocated buffer (not in-place), so the chunk index
   * that the STFT element packed into GST_BUFFER_OFFSET (see
   * gstdspkernel.cpp) would otherwise be lost here and never reach
   * interleave/istft downstream. Carry it forward explicitly. */
  GST_BUFFER_OFFSET (*outbuf) = GST_BUFFER_OFFSET (inbuf);

  return GST_FLOW_OK;
}

static GstFlowReturn
gst_ti_tvm_transform (GstBaseTransform * trans, GstBuffer * inbuf,
    GstBuffer * outbuf)
{
  GstTiTvm *tvm = GST_TI_TVM (trans);
  GstMapInfo in_map, out_map;
  GstFlowReturn ret;

  /* Map input buffer */
  if (!gst_buffer_map (inbuf, &in_map, GST_MAP_READ)) {
    GST_ERROR_OBJECT (tvm, "Failed to map input buffer");
    return GST_FLOW_ERROR;
  }

  GST_DEBUG_OBJECT (tvm, "transform: input buffer size=%zu bytes", in_map.size);

  /* Handle empty buffers (STFT accumulation phase) - pass through immediately */
  if (in_map.size == 0) {
    GST_DEBUG_OBJECT (tvm, "Empty buffer, passing through");
    gst_buffer_unmap (inbuf, &in_map);
    gst_buffer_set_size (outbuf, 0);
    return GST_FLOW_OK;
  }

  gfloat *input_data = (gfloat *) in_map.data;
  gsize input_size = in_map.size / sizeof (gfloat);

  guint64 offset = GST_BUFFER_OFFSET (inbuf);
  if (offset != GST_BUFFER_OFFSET_NONE) {
    gsize chunk_idx = (offset >> 16) & 0xFFFF;
    gsize n_chunks = offset & 0xFFFF;

    GST_INFO_OBJECT (tvm, "transform: chunk %zu/%zu, %zu floats (%zu bytes)",
        chunk_idx + 1, n_chunks, input_size, in_map.size);
  } else {
    GST_DEBUG_OBJECT (tvm, "transform: %zu floats (%zu bytes)", input_size,
        in_map.size);
  }


  /* Run inference benchmark */
  ret = gst_ti_tvm_run_inference (tvm, input_data, input_size);

  /* Copy inference output to output buffer */
  if (ret == GST_FLOW_OK && tvm->output_num_floats > 0 &&
      (tvm->daemon_fd >= 0 || tvm->final_output)) {
    gsize out_bytes = tvm->output_num_floats * sizeof (float);

    /* Map output buffer (write-only) */
    if (!gst_buffer_map (outbuf, &out_map, GST_MAP_WRITE)) {
      GST_ERROR_OBJECT (tvm, "Failed to map output buffer");
      gst_buffer_unmap (inbuf, &in_map);
      return GST_FLOW_ERROR;
    }

    /* Copy inference output to buffer, from the daemon or from the
     * in-process TVM graph executor depending on which path ran. */
    if (tvm->daemon_fd >= 0) {
      memcpy (out_map.data, tvm->daemon_output_buf, out_bytes);
    } else {
      NDArray *out = (NDArray *) tvm->final_output;
      out->CopyToBytes (out_map.data, out_bytes);
    }

    /* Optional: live top-k class prediction printing (no-op unless
     * class-map-path was configured, e.g. for classification models). */
    gst_ti_tvm_print_top_predictions (tvm, inbuf, (const gfloat *) out_map.data,
        tvm->output_num_floats);

    gst_buffer_unmap (outbuf, &out_map);

    /* Set actual output size */
    gst_buffer_set_size (outbuf, (gssize) out_bytes);
  }

  gst_buffer_unmap (inbuf, &in_map);

  if (ret != GST_FLOW_OK)
    GST_ERROR_OBJECT (tvm, "Inference failed");

  return ret;
}

static gboolean
gst_ti_tvm_load_model (GstTiTvm * tvm)
{
  try {
    gchar *graph_path =
        g_strdup_printf ("%s/deploy_graph.json", tvm->model_path);

    GST_INFO_OBJECT (tvm, "[TVM] Initializing with artifacts from: %s",
        tvm->model_path);

    gchar *graph_json = gst_ti_tvm_load_json_file (graph_path);

    if (!graph_json) {
      GST_ERROR_OBJECT (tvm, "Failed to load graph JSON from %s", graph_path);
      g_free (graph_path);
      return FALSE;
    }

    /* Auto-detect input/output shapes from deploy_graph.json. This is
     * needed regardless of which inference path (daemon or in-process)
     * ends up running. */
    std::vector < int64_t > json_input_shape;
    std::vector < int64_t > json_output_shape;
    gchar *json_input_name = NULL;

    gboolean shape_parsed =
        gst_ti_tvm_parse_shape_from_json (tvm, graph_json, json_input_shape,
        json_output_shape, &json_input_name);

    if (shape_parsed) {
      *static_cast < std::vector < int64_t > *>(tvm->auto_input_shape) =
          json_input_shape;
      *static_cast < std::vector < int64_t > *>(tvm->auto_output_shape) =
          json_output_shape;
      if (json_input_name) {
        tvm->auto_input_name = json_input_name;
      }
      GST_INFO_OBJECT (tvm, "[TVM] Shape auto-detection: SUCCESS");
    } else {
      GST_WARNING_OBJECT (tvm,
          "[TVM] Shape auto-detection: FAILED (will require input-shape property)");
      g_print ("[TVM Model Shape Detection]\n");
      g_print
          ("  Shape auto-detection FAILED - will use property or 1D shape\n");
    }

    g_free (graph_path);

    /* Prefer the shared tvm-model-daemon: the DSP compute channel only
     * supports one client, and on boards running the daemon it already
     * owns that session. Loading the graph executor in-process here would
     * make a second, competing c7x_client_open() call and fail. */
    tvm->daemon_fd = gst_ti_tvm_daemon_connect (tvm);
    if (tvm->daemon_fd >= 0) {
      g_free (graph_json);
      tvm->tvm_initialized = TRUE;
      return TRUE;
    }

    GST_INFO_OBJECT (tvm,
        "[TVM] Model daemon unavailable, loading model in-process");

    gchar *lib_path = g_strdup_printf ("%s/deploy_lib.so", tvm->model_path);
    gchar *param_path =
        g_strdup_printf ("%s/deploy_param.params", tvm->model_path);

    /* Load TVM artifacts */
    Module lib = Module::LoadFromFile (lib_path);

    auto graph_executor_create = Registry::Get ("tvm.graph_executor.create");
    if (!graph_executor_create) {
      GST_ERROR_OBJECT (tvm, "tvm.graph_executor.create not found in registry");
      g_free (lib_path);
      g_free (param_path);
      g_free (graph_json);
      return FALSE;
    }

    Module executor;
    try {
      executor =
          (*graph_executor_create) (String (graph_json), lib, int (kDLCPU),
          int (0));
    } catch (const std::exception & e1)
    {
      GST_ERROR_OBJECT (tvm, "graph_executor.create failed: %s", e1.what ());
      g_free (lib_path);
      g_free (param_path);
      g_free (graph_json);
      return FALSE;
    }

    /* Load parameters */
    gsize param_size;
    gchar *param_data = gst_ti_tvm_load_param_file (param_path, &param_size);
    if (!param_data) {
      GST_ERROR_OBJECT (tvm, "Failed to load parameters from %s", param_path);
      g_free (lib_path);
      g_free (param_path);
      g_free (graph_json);
      return FALSE;
    }

    GST_INFO_OBJECT (tvm, "[TVM] Loading parameters: %s", param_path);
    auto load_params = executor.GetFunction ("load_params");
    TVMByteArray param_array = { param_data, param_size };
    load_params (param_array);
    GST_INFO_OBJECT (tvm, "[TVM] Parameters loaded (%zu bytes)", param_size);

    tvm->graph_executor = new Module (executor);
    PackedFunc *set_input = new PackedFunc (executor.GetFunction ("set_input"));
    PackedFunc *run = new PackedFunc (executor.GetFunction ("run"));
    PackedFunc *get_output =
        new PackedFunc (executor.GetFunction ("get_output"));

    tvm->set_input_func = set_input;
    tvm->run_func = run;
    tvm->get_output_func = get_output;

    GST_INFO_OBJECT (tvm,
        "[TVM] Input configuration: index=0 (auto-detected from JSON)");

    tvm->tvm_initialized = TRUE;

    g_free (lib_path);
    g_free (param_path);
    g_free (graph_json);
    g_free (param_data);

    return TRUE;

  }
  catch (const std::exception & e)
  {
    GST_ERROR_OBJECT (tvm, "TVM initialization failed: %s", e.what ());
    return FALSE;
  }
}

/* Read/write exactly size bytes, looping over short reads/writes (a
 * Unix stream socket may legitimately transfer less than requested in
 * a single call, e.g. for buffers larger than the socket's buffer size).
 * Returns FALSE on EOF/error before size bytes were transferred. */
static gboolean
gst_ti_tvm_daemon_read_full (gint fd, gpointer buf, gsize size)
{
  gsize total = 0;

  while (total < size) {
    gssize n = read (fd, (guint8 *) buf + total, size - total);
    if (n <= 0)
      return FALSE;
    total += n;
  }

  return TRUE;
}

static gboolean
gst_ti_tvm_daemon_write_full (gint fd, gconstpointer buf, gsize size)
{
  gsize total = 0;

  while (total < size) {
    gssize n = write (fd, (const guint8 *) buf + total, size - total);
    if (n <= 0)
      return FALSE;
    total += n;
  }

  return TRUE;
}

/* Connect to tvm-model-daemon and perform the PING/PONG handshake.
 * Returns a connected fd, or -1 if the daemon is not available. */
static gint
gst_ti_tvm_daemon_connect (GstTiTvm * tvm)
{
  struct sockaddr_un addr;
  struct TvmDaemonHeader hdr;
  gint fd;

  fd = socket (AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    GST_WARNING_OBJECT (tvm, "[TVM] Failed to create daemon socket: %s",
        g_strerror (errno));
    return -1;
  }

  memset (&addr, 0, sizeof (addr));
  addr.sun_family = AF_UNIX;
  g_strlcpy (addr.sun_path, TVM_DAEMON_SOCKET_PATH, sizeof (addr.sun_path));

  if (connect (fd, (struct sockaddr *) &addr, sizeof (addr)) < 0) {
    GST_INFO_OBJECT (tvm, "[TVM] Model daemon not reachable at %s: %s",
        TVM_DAEMON_SOCKET_PATH, g_strerror (errno));
    close (fd);
    return -1;
  }

  hdr.magic = TVM_DAEMON_MAGIC;
  hdr.type = TVM_DAEMON_MSG_PING;
  hdr.len = 0;

  if (!gst_ti_tvm_daemon_write_full (fd, &hdr, sizeof (hdr)) ||
      !gst_ti_tvm_daemon_read_full (fd, &hdr, sizeof (hdr)) ||
      hdr.magic != TVM_DAEMON_MAGIC || hdr.type != TVM_DAEMON_MSG_PONG) {
    GST_WARNING_OBJECT (tvm, "[TVM] Model daemon handshake failed");
    close (fd);
    return -1;
  }

  GST_INFO_OBJECT (tvm, "[TVM] Connected to model daemon at %s",
      TVM_DAEMON_SOCKET_PATH);

  return fd;
}

static GstFlowReturn
gst_ti_tvm_run_inference_daemon (GstTiTvm * tvm, gfloat * input_data,
    gsize input_size)
{
  struct TvmDaemonHeader req_hdr, resp_hdr;
  gsize payload_bytes = input_size * sizeof (gfloat);

  req_hdr.magic = TVM_DAEMON_MAGIC;
  req_hdr.type = TVM_DAEMON_MSG_INFER_REQ;
  req_hdr.len = (uint32_t) payload_bytes;

  GST_INFO_OBJECT (tvm,
      "[TVM] Daemon request: %zu floats (%zu bytes), input[0..3]=%.6f,%.6f,%.6f,%.6f",
      input_size, payload_bytes, input_data[0], input_data[1], input_data[2],
      input_data[3]);

  if (!gst_ti_tvm_daemon_write_full (tvm->daemon_fd, &req_hdr,
          sizeof (req_hdr)) ||
      !gst_ti_tvm_daemon_write_full (tvm->daemon_fd, input_data,
          payload_bytes)) {
    GST_ERROR_OBJECT (tvm, "[TVM] Failed to send inference request: %s",
        g_strerror (errno));
    return GST_FLOW_ERROR;
  }

  if (!gst_ti_tvm_daemon_read_full (tvm->daemon_fd, &resp_hdr,
          sizeof (resp_hdr)) || resp_hdr.magic != TVM_DAEMON_MAGIC) {
    GST_ERROR_OBJECT (tvm, "[TVM] Invalid response header from daemon");
    return GST_FLOW_ERROR;
  }

  if (resp_hdr.type == TVM_DAEMON_MSG_ERROR_RESP) {
    gchar *err_msg = g_new0 (gchar, resp_hdr.len + 1);

    gst_ti_tvm_daemon_read_full (tvm->daemon_fd, err_msg, resp_hdr.len);
    GST_ERROR_OBJECT (tvm, "[TVM] Daemon inference error: %s", err_msg);
    g_free (err_msg);
    return GST_FLOW_ERROR;
  }

  if (resp_hdr.type != TVM_DAEMON_MSG_INFER_RESP ||
      resp_hdr.len % sizeof (gfloat) != 0) {
    GST_ERROR_OBJECT (tvm,
        "[TVM] Unexpected response from daemon (type=%u len=%u)",
        resp_hdr.type, resp_hdr.len);
    return GST_FLOW_ERROR;
  }

  gsize out_floats = resp_hdr.len / sizeof (gfloat);

  if (out_floats != input_size) {
    GST_WARNING_OBJECT (tvm,
        "[TVM] Daemon returned %zu floats, expected %zu (same-shape model) "
        "- output/interleave buffers may be misaligned", out_floats,
        input_size);
  }

  if (out_floats > tvm->daemon_output_buf_size) {
    g_free (tvm->daemon_output_buf);
    tvm->daemon_output_buf = g_new (gfloat, out_floats);
    tvm->daemon_output_buf_size = out_floats;
  }

  if (!gst_ti_tvm_daemon_read_full (tvm->daemon_fd, tvm->daemon_output_buf,
          resp_hdr.len)) {
    GST_ERROR_OBJECT (tvm, "[TVM] Failed to read inference response payload");
    return GST_FLOW_ERROR;
  }

  GST_INFO_OBJECT (tvm,
      "[TVM] Daemon response: %zu floats (%u bytes), output[0..3]=%.6f,%.6f,%.6f,%.6f",
      out_floats, resp_hdr.len, tvm->daemon_output_buf[0],
      tvm->daemon_output_buf[1], tvm->daemon_output_buf[2],
      tvm->daemon_output_buf[3]);

  tvm->output_num_floats = out_floats;
  tvm->inference_completed = TRUE;

  return GST_FLOW_OK;
}

static GstFlowReturn
gst_ti_tvm_run_inference (GstTiTvm * tvm, gfloat * input_data, gsize input_size)
{
  if (tvm->daemon_fd >= 0) {
    return gst_ti_tvm_run_inference_daemon (tvm, input_data, input_size);
  }

  try {
    PackedFunc *set_input = (PackedFunc *) tvm->set_input_func;
    PackedFunc *run = (PackedFunc *) tvm->run_func;
    PackedFunc *get_output = (PackedFunc *) tvm->get_output_func;

    /* Determine input shape: auto-detect from JSON */
    std::vector < int64_t > shape;

    std::vector < int64_t > *auto_shape_ptr =
        static_cast < std::vector < int64_t > *>(tvm->auto_input_shape);
    if (auto_shape_ptr && !auto_shape_ptr->empty ()) {
      shape = *auto_shape_ptr;

      /* Validate auto-detected shape matches input data size */
      gsize shape_elements = 1;
      for (size_t i = 0; i < shape.size (); i++) {
        shape_elements *= shape[i];
      }
      if (shape_elements != input_size) {
        GST_ERROR_OBJECT (tvm,
            "[TVM] Auto-detected shape requires %zu elements but got %zu floats",
            shape_elements, input_size);
        return GST_FLOW_ERROR;
      }

      if (shape.size () == 4) {
        GST_INFO_OBJECT (tvm,
            "[TVM] Using auto-detected shape: [%ld,%ld,%ld,%ld]", shape[0],
            shape[1], shape[2], shape[3]);
      } else {
        GST_INFO_OBJECT (tvm, "[TVM] Using auto-detected shape");
      }
    } else {
      /* Default: flat 1D shape */
      shape = { static_cast < int64_t > (input_size) };
      GST_INFO_OBJECT (tvm,
          "[TVM] No input shape auto-detected, using flat 1D shape: [%zu]",
          input_size);
    }

    /* Create input NDArray */
    NDArray input_array = NDArray::Empty (shape, DLDataType {
          kDLFloat, 32, 1
        }, DLDevice {
          kDLCPU, 0
        }
    );
    input_array.CopyFromBytes (input_data, input_size * sizeof (float));

    /* Run single inference */
    auto start_time = std::chrono::high_resolution_clock::now ();

    (*set_input) (0, input_array);
    (*run) ();
    NDArray output_array = (*get_output) (0);

    auto end_time = std::chrono::high_resolution_clock::now ();
    auto duration =
        std::chrono::duration_cast < std::chrono::microseconds >
        (end_time - start_time);
    gdouble time_ms = duration.count () / 1000.0;

    tvm->perf_data.first_run_time = duration.count ();

    /* Get output shape */
    gsize out_floats = 1;
    for (int j = 0; j < output_array->ndim; j++) {
      out_floats *= output_array->shape[j];
    }
    tvm->output_num_floats = out_floats;


    /* Store output for returning to pipeline */
    if (tvm->final_output) {
      delete (NDArray *) tvm->final_output;
    }
    tvm->final_output = new NDArray (output_array);

    tvm->inference_completed = TRUE;
    return GST_FLOW_OK;

  }
  catch (const std::exception & e)
  {
    GST_ERROR_OBJECT (tvm, "Inference failed: %s", e.what ());
    return GST_FLOW_ERROR;
  }
}

/* Helper functions */

static gboolean
gst_ti_tvm_parse_shape_from_json (GstTiTvm * tvm, const gchar * graph_json,
    std::vector < int64_t > &input_shape, std::vector < int64_t > &output_shape,
    gchar ** input_name)
{
  struct json_object *root_obj = NULL;
  struct json_object *attrs_obj = NULL;
  struct json_object *shape_array = NULL;
  struct json_object *nodes_array = NULL;
  struct json_object *arg_nodes_array = NULL;
  struct json_object *shapes = NULL;
  struct json_object *input_shape_array = NULL;
  struct json_object *output_shape_array = NULL;
  struct json_object *first_arg_node = NULL;
  struct json_object *input_node = NULL;
  struct json_object *name_obj = NULL;
  gboolean ret = FALSE;

  root_obj = json_tokener_parse (graph_json);
  if (!root_obj) {
    GST_WARNING_OBJECT (tvm, "Failed to parse JSON");
    goto cleanup;
  }

  /* Extract input tensor name from nodes array */
  if (json_object_object_get_ex (root_obj, "nodes", &nodes_array) &&
      json_object_object_get_ex (root_obj, "arg_nodes", &arg_nodes_array)) {
    first_arg_node = json_object_array_get_idx (arg_nodes_array, 0);
    if (first_arg_node) {
      gint input_node_idx = json_object_get_int (first_arg_node);
      input_node = json_object_array_get_idx (nodes_array, input_node_idx);
      if (input_node) {
        if (json_object_object_get_ex (input_node, "name", &name_obj)) {
          const gchar *name = json_object_get_string (name_obj);
          *input_name = g_strdup (name);
          GST_DEBUG_OBJECT (tvm, "[JSON] Input tensor name: %s", *input_name);
        }
      }
    }
  }

  /* Get attrs object */
  if (!json_object_object_get_ex (root_obj, "attrs", &attrs_obj)) {
    GST_WARNING_OBJECT (tvm, "JSON missing 'attrs' field");
    goto cleanup;
  }

  /* Get shape array */
  if (!json_object_object_get_ex (attrs_obj, "shape", &shape_array)) {
    GST_WARNING_OBJECT (tvm, "JSON missing 'attrs.shape' field");
    goto cleanup;
  }

  /* Shape array format: ["list_shape", [[input_shape], [output_shape]]] */
  if (json_object_array_length (shape_array) < 2) {
    GST_WARNING_OBJECT (tvm, "Invalid shape array length");
    goto cleanup;
  }

  /* Get the actual shapes array (second element) */
  shapes = json_object_array_get_idx (shape_array, 1);
  if (!shapes) {
    GST_WARNING_OBJECT (tvm, "Failed to get shapes array");
    goto cleanup;
  }

  if (json_object_array_length (shapes) < 2) {
    GST_WARNING_OBJECT (tvm,
        "Invalid shapes array, expected at least 2 elements");
    goto cleanup;
  }

  /* Parse input shape (first element) */
  input_shape_array = json_object_array_get_idx (shapes, 0);
  if (input_shape_array) {
    size_t input_shape_len = json_object_array_length (input_shape_array);
    for (size_t i = 0; i < input_shape_len; i++) {
      struct json_object *dim_obj =
          json_object_array_get_idx (input_shape_array, i);
      gint64 dim = json_object_get_int64 (dim_obj);
      input_shape.push_back (dim);
    }
  }

  /* Parse output shape (second element) */
  output_shape_array = json_object_array_get_idx (shapes, 1);
  if (output_shape_array) {
    size_t output_shape_len = json_object_array_length (output_shape_array);
    for (size_t i = 0; i < output_shape_len; i++) {
      struct json_object *dim_obj =
          json_object_array_get_idx (output_shape_array, i);
      gint64 dim = json_object_get_int64 (dim_obj);
      output_shape.push_back (dim);
    }
  }

  if (input_shape.size () == 4) {
    GST_INFO_OBJECT (tvm, "[JSON] Auto-detected input shape: [%ld,%ld,%ld,%ld]",
        input_shape[0], input_shape[1], input_shape[2], input_shape[3]);
  }

  if (output_shape.size () == 4) {
    GST_INFO_OBJECT (tvm,
        "[JSON] Auto-detected output shape: [%ld,%ld,%ld,%ld]",
        output_shape[0], output_shape[1], output_shape[2], output_shape[3]);
  }

  ret = TRUE;

cleanup:
  if (root_obj) {
    json_object_put (root_obj);
  }
  return ret;
}

static gchar *
gst_ti_tvm_load_json_file (const gchar * file_path)
{
  GError *error = NULL;
  gchar *contents = NULL;
  gsize length;

  if (!g_file_get_contents (file_path, &contents, &length, &error)) {
    g_warning ("Failed to read file %s: %s", file_path, error->message);
    g_error_free (error);
    return NULL;
  }

  return contents;
}

static gchar *
gst_ti_tvm_load_param_file (const gchar * file_path, gsize * file_size)
{
  GError *error = NULL;
  gchar *contents = NULL;

  if (!g_file_get_contents (file_path, &contents, file_size, &error)) {
    g_warning ("Failed to read parameter file %s: %s", file_path,
        error->message);
    g_error_free (error);
    return NULL;
  }

  return contents;
}

/* Parse an ordered list of class names out of a YAML file shaped like
 * yamnet_class_map.yml:
 *   - id: /m/09x0r
 *     name: Speech
 *   - id: /m/0ytgt
 *     name: Child speech, kid speaking
 * Only the "name:" scalar on each entry is needed (in file order, which is
 * the class index order the model output uses) - avoids pulling in a full
 * YAML parser for this one flat, non-nested structure. No-op (leaves
 * class_names NULL) when class_map_path is unset, so printing stays
 * disabled by default for non-classification models. */
static void
gst_ti_tvm_load_class_map (GstTiTvm * tvm)
{
  gchar *contents;
  gchar **lines;
  GPtrArray *names;
  guint i;

  if (tvm->class_names) {
    for (i = 0; i < tvm->num_class_names; i++)
      g_free (tvm->class_names[i]);
    g_free (tvm->class_names);
    tvm->class_names = NULL;
    tvm->num_class_names = 0;
  }

  if (!tvm->class_map_path || tvm->class_map_path[0] == '\0')
    return;

  contents = gst_ti_tvm_load_json_file (tvm->class_map_path);
  if (!contents) {
    GST_WARNING_OBJECT (tvm, "[TVM] Failed to load class-map-path: %s",
        tvm->class_map_path);
    return;
  }

  names = g_ptr_array_new ();
  lines = g_strsplit (contents, "\n", -1);
  for (i = 0; lines[i] != NULL; i++) {
    gchar *line = g_strstrip (lines[i]);
    if (g_str_has_prefix (line, "name:")) {
      gchar *name = g_strstrip (line + strlen ("name:"));
      g_ptr_array_add (names, g_strdup (name));
    }
  }
  g_strfreev (lines);
  g_free (contents);

  tvm->num_class_names = names->len;
  tvm->class_names = (gchar **) g_ptr_array_free (names, FALSE);

  GST_INFO_OBJECT (tvm, "[TVM] Loaded %u class names from %s",
      tvm->num_class_names, tvm->class_map_path);
  g_print
      ("[TVM] Live top-%u prediction printing enabled (%u classes from %s)\n",
      tvm->top_k, tvm->num_class_names, tvm->class_map_path);
}

/* Print the top-k class predictions for one inference window. No-op when
 * class-map-path wasn't configured (tvm->class_names == NULL). */
static void
gst_ti_tvm_print_top_predictions (GstTiTvm * tvm, GstBuffer * inbuf,
    const gfloat * scores, gsize count)
{
  guint top_k;
  gsize *indices;
  gboolean *used;
  gsize i, j;
  guint64 offset;

  if (!tvm->class_names || tvm->num_class_names == 0)
    return;

  top_k = MIN (tvm->top_k, (guint) count);
  indices = g_new (gsize, top_k);
  used = g_new0 (gboolean, count);

  /* Simple partial selection: count/top_k are both small (<=521, <=10). */
  for (i = 0; i < top_k; i++) {
    gsize best = G_MAXSIZE;
    for (j = 0; j < count; j++) {
      if (used[j])
        continue;
      if (best == G_MAXSIZE || scores[j] > scores[best])
        best = j;
    }
    if (best == G_MAXSIZE)
      break;
    used[best] = TRUE;
    indices[i] = best;
  }

  tvm->window_counter++;

  offset = GST_BUFFER_OFFSET (inbuf);
  if (offset != GST_BUFFER_OFFSET_NONE) {
    gsize chunk_idx = (offset >> 16) & 0xFFFF;
    gsize n_chunks = offset & 0xFFFF;
    g_print ("  Top-%u predictions for window %zu/%zu:\n", top_k,
        chunk_idx + 1, n_chunks);
  } else {
    g_print ("  Top-%u predictions for window %u:\n", top_k,
        tvm->window_counter);
  }

  for (i = 0; i < top_k; i++) {
    gsize idx = indices[i];
    const gchar *name =
        idx < tvm->num_class_names ? tvm->class_names[idx] : "Unknown";
    g_print ("    %zu. %s: %.6f\n", i + 1, name, scores[idx]);
  }

  g_free (used);
  g_free (indices);
}
