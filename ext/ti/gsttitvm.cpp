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

using namespace
    tvm::runtime;

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
    GstFlowReturn
gst_ti_tvm_run_inference_benchmark (GstTiTvm * tvm,
    gfloat * input_data, gsize input_size);

static void
gst_ti_tvm_print_performance_stats (GstTiTvm * tvm);

static gchar *
gst_ti_tvm_load_json_file (const gchar * file_path);

static gchar *
gst_ti_tvm_load_param_file (const gchar * file_path, gsize * file_size);

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

  base_transform_class->passthrough_on_same_caps = TRUE;

  /* Properties */
  g_object_class_install_property (gobject_class, PROP_MODEL_PATH,
      g_param_spec_string ("model-path", "Model Path",
          "Path to TVM artifacts directory", DEFAULT_MODEL_PATH,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_ITERATIONS,
      g_param_spec_int ("iterations", "Iterations",
          "Number of inference iterations to run", 1, 1000, DEFAULT_ITERATIONS,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_BENCHMARK,
      g_param_spec_boolean ("benchmark", "Benchmark",
          "Enable performance benchmarking output", DEFAULT_BENCHMARK,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}

static void
gst_ti_tvm_init (GstTiTvm * tvm)
{
  tvm->model_path = g_strdup (DEFAULT_MODEL_PATH);
  tvm->iterations = DEFAULT_ITERATIONS;
  tvm->benchmark = DEFAULT_BENCHMARK;

  tvm->tvm_initialized = FALSE;
  tvm->graph_executor = NULL;
  tvm->set_input_func = NULL;
  tvm->run_func = NULL;
  tvm->get_output_func = NULL;

  tvm->final_output = NULL;
  tvm->output_num_floats = 0;

  memset (&tvm->perf_data, 0, sizeof (tvm->perf_data));

  tvm->inference_completed = FALSE;
  tvm->current_iteration = 0;
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
    case PROP_ITERATIONS:
      tvm->iterations = g_value_get_int (value);
      break;
    case PROP_BENCHMARK:
      tvm->benchmark = g_value_get_boolean (value);
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
    case PROP_ITERATIONS:
      g_value_set_int (value, tvm->iterations);
      break;
    case PROP_BENCHMARK:
      g_value_set_boolean (value, tvm->benchmark);
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

  if (tvm->perf_data.inference_times) {
    g_free (tvm->perf_data.inference_times);
    tvm->perf_data.inference_times = NULL;
  }

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static
    gboolean
gst_ti_tvm_start (GstBaseTransform * trans)
{
  GstTiTvm *
      tvm = GST_TI_TVM (trans);

  GST_DEBUG_OBJECT (tvm, "Starting TI TVM inference element");
  g_print ("TI TVM Inference Element\n");
  g_print ("========================\n");

  /* Initialize TVM runtime */
  if (!gst_ti_tvm_load_model (tvm)) {
    GST_ERROR_OBJECT (tvm, "Failed to load TVM model");
    return FALSE;
  }

  /* Allocate performance tracking arrays */
  tvm->perf_data.inference_times = g_new0 (gint64, tvm->iterations);

  GST_DEBUG_OBJECT (tvm, "TI TVM element started successfully");

  return TRUE;
}

static
    gboolean
gst_ti_tvm_stop (GstBaseTransform * trans)
{
  GstTiTvm *
      tvm = GST_TI_TVM (trans);

  GST_DEBUG_OBJECT (tvm, "Stopping TI TVM inference element");

  tvm->tvm_initialized = FALSE;

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
  GstCaps *
      othercaps;

  othercaps = gst_caps_new_simple ("application/octet-stream", NULL, NULL);

  if (filter) {
    GstCaps *
        intersect = gst_caps_intersect_full (othercaps, filter,
        GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (othercaps);
    othercaps = intersect;
  }

  return othercaps;
}

/* Prepare output buffer with correct size */
static
    GstFlowReturn
gst_ti_tvm_prepare_output_buffer (GstBaseTransform * trans, GstBuffer * inbuf,
    GstBuffer ** outbuf)
{
  GstTiTvm *
      tvm = GST_TI_TVM (trans);
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

  return GST_FLOW_OK;
}

static
    GstFlowReturn
gst_ti_tvm_transform (GstBaseTransform * trans, GstBuffer * inbuf,
    GstBuffer * outbuf)
{
  GstTiTvm *
      tvm = GST_TI_TVM (trans);
  GstMapInfo in_map, out_map;
  GstFlowReturn ret;

  /* Map input buffer */
  if (!gst_buffer_map (inbuf, &in_map, GST_MAP_READ)) {
    GST_ERROR_OBJECT (tvm, "Failed to map input buffer");
    return GST_FLOW_ERROR;
  }

  gfloat *
      input_data = (gfloat *) in_map.data;
  gsize input_size = in_map.size / sizeof (gfloat);

  GST_DEBUG_OBJECT (tvm, "transform: %zu floats (%zu bytes)", input_size,
      in_map.size);

  /* Run inference benchmark */
  ret = gst_ti_tvm_run_inference_benchmark (tvm, input_data, input_size);

  /* Copy inference output to output buffer */
  if (ret == GST_FLOW_OK && tvm->final_output && tvm->output_num_floats > 0) {
    NDArray *
        out = (NDArray *) tvm->final_output;
    gsize out_bytes = tvm->output_num_floats * sizeof (float);

    /* Map output buffer (write-only) */
    if (!gst_buffer_map (outbuf, &out_map, GST_MAP_WRITE)) {
      GST_ERROR_OBJECT (tvm, "Failed to map output buffer");
      gst_buffer_unmap (inbuf, &in_map);
      return GST_FLOW_ERROR;
    }

    /* Copy output data to output buffer (sized by prepare_output_buffer) */
    out->CopyToBytes (out_map.data, out_bytes);
    gst_buffer_unmap (outbuf, &out_map);

    /* Set actual output size */
    gst_buffer_set_size (outbuf, (gssize) out_bytes);
  }

  /* Unmap input buffer */
  gst_buffer_unmap (inbuf, &in_map);

  if (ret != GST_FLOW_OK)
    GST_ERROR_OBJECT (tvm, "Inference failed");

  return ret;
}

static
    gboolean
gst_ti_tvm_load_model (GstTiTvm * tvm)
{
  try {
    gchar *
        lib_path = g_strdup_printf ("%s/deploy_lib.so", tvm->model_path);
    gchar *
        graph_path = g_strdup_printf ("%s/deploy_graph.json", tvm->model_path);
    gchar *
        param_path =
        g_strdup_printf ("%s/deploy_param.params", tvm->model_path);

    GST_INFO_OBJECT (tvm, "[TVM] Initializing with artifacts from: %s",
        tvm->model_path);

    /* Load TVM artifacts (same as C application) */
    Module lib = Module::LoadFromFile (lib_path);
    gchar *
        graph_json = gst_ti_tvm_load_json_file (graph_path);

    if (!graph_json) {
      GST_ERROR_OBJECT (tvm, "Failed to load graph JSON from %s", graph_path);
      g_free (lib_path);
      g_free (graph_path);
      g_free (param_path);
      return FALSE;
    }

    auto graph_executor_create = Registry::Get ("tvm.graph_executor.create");
    if (!graph_executor_create) {
      GST_ERROR_OBJECT (tvm, "tvm.graph_executor.create not found in registry");
      g_free (lib_path);
      g_free (graph_path);
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
      g_free (graph_path);
      g_free (param_path);
      g_free (graph_json);
      return FALSE;
    }

    /* Load parameters */
    gsize param_size;
    gchar *
        param_data = gst_ti_tvm_load_param_file (param_path, &param_size);
    if (!param_data) {
      GST_ERROR_OBJECT (tvm, "Failed to load parameters from %s", param_path);
      g_free (lib_path);
      g_free (graph_path);
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
    PackedFunc *
        set_input = new PackedFunc (executor.GetFunction ("set_input"));
    PackedFunc *
        run = new PackedFunc (executor.GetFunction ("run"));
    PackedFunc *
        get_output = new PackedFunc (executor.GetFunction ("get_output"));

    tvm->set_input_func = set_input;
    tvm->run_func = run;
    tvm->get_output_func = get_output;

    GST_INFO_OBJECT (tvm,
        "[TVM] Input configuration: index=0, shape=[dynamic]");

    tvm->tvm_initialized = TRUE;

    g_free (lib_path);
    g_free (graph_path);
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

static
    GstFlowReturn
gst_ti_tvm_run_inference_benchmark (GstTiTvm * tvm, gfloat * input_data,
    gsize input_size)
{
  try {
    PackedFunc *
        set_input = (PackedFunc *) tvm->set_input_func;
    PackedFunc *
        run = (PackedFunc *) tvm->run_func;
    PackedFunc *
        get_output = (PackedFunc *) tvm->get_output_func;

    /* Create input NDArray once (reused across iterations) */
    std::vector < int64_t > shape = { static_cast < int64_t > (input_size) };
    NDArray input_array = NDArray::Empty (shape, DLDataType {
        kDLFloat, 32, 1}, DLDevice {
        kDLCPU, 0});
    input_array.CopyFromBytes (input_data, input_size * sizeof (float));

    if (tvm->benchmark) {
      g_print ("\n[TVM] Running inference benchmark (%d iterations)...\n",
          tvm->iterations);
      g_print ("------------------------------\n");
    }

    /* Run inference iterations */
    for (gint i = 0; i < tvm->iterations; i++) {
      auto start_time = std::chrono::high_resolution_clock::now ();

      (*set_input) (0, input_array);
      (*run) ();
      NDArray output_array = (*get_output) (0);

      auto end_time = std::chrono::high_resolution_clock::now ();
      auto duration =
          std::chrono::duration_cast < std::chrono::microseconds >
          (end_time - start_time);
      gdouble time_ms = duration.count () / 1000.0;

      if (i == 0) {
        /* First run includes initialization overhead */
        tvm->perf_data.first_run_time = duration.count ();
        if (tvm->benchmark) {
          g_print ("First run (includes init): %.2f ms\n", time_ms);
        }

        /* Get output shape from first run */
        gsize out_floats = 1;
        for (int j = 0; j < output_array->ndim; j++) {
          out_floats *= output_array->shape[j];
        }
        tvm->output_num_floats = out_floats;

        if (tvm->benchmark) {
          g_print ("Output shape: (");
          for (int j = 0; j < output_array->ndim; j++) {
            g_print ("%s%ld", (j > 0) ? ", " : "", output_array->shape[j]);
          }
          g_print (")\n");
        }
      } else {
        /* Subsequent runs for performance measurement */
        tvm->perf_data.inference_times[i - 1] = duration.count ();
        if (tvm->benchmark) {
          g_print ("Run %d: %.2f ms\n", i + 1, time_ms);
        }
      }

      /* Store final output for returning to pipeline */
      if (i == tvm->iterations - 1) {
        if (tvm->final_output) {
          delete (NDArray *) tvm->final_output;
        }
        tvm->final_output = new NDArray (output_array);
      }
    }

    /* Calculate and print performance statistics */
    if (tvm->benchmark && tvm->iterations > 1) {
      gst_ti_tvm_print_performance_stats (tvm);
    }

    tvm->inference_completed = TRUE;
    return GST_FLOW_OK;

  }
  catch (const std::exception & e)
  {
    GST_ERROR_OBJECT (tvm, "Inference benchmark failed: %s", e.what ());
    return GST_FLOW_ERROR;
  }
}

static void
gst_ti_tvm_print_performance_stats (GstTiTvm * tvm)
{
  gint num_runs = tvm->iterations - 1;  /* Exclude first run */

  if (num_runs <= 0) {
    return;
  }

  gdouble sum = 0.0;
  gdouble min_time = G_MAXDOUBLE;
  gdouble max_time = 0.0;

  for (gint i = 0; i < num_runs; i++) {
    gdouble time_ms = tvm->perf_data.inference_times[i] / 1000.0;
    sum += time_ms;
    if (time_ms < min_time)
      min_time = time_ms;
    if (time_ms > max_time)
      max_time = time_ms;
  }

  gdouble avg_time = sum / num_runs;
  gdouble fps = 1000.0 / avg_time;

  tvm->perf_data.avg_time = avg_time;
  tvm->perf_data.min_time = min_time;
  tvm->perf_data.max_time = max_time;
  tvm->perf_data.fps = fps;

  g_print ("\n");
  g_print ("Performance Results:\n");
  g_print ("  Average: %.2f ms\n", avg_time);
  g_print ("  Min:     %.2f ms\n", min_time);
  g_print ("  Max:     %.2f ms\n", max_time);
  g_print ("  FPS:     %.1f\n", fps);
  g_print ("\n");
  g_print ("Inference completed successfully!\n");
  g_print ("  Average inference time: %.2f ms\n", avg_time);
  g_print ("  Using TVM+TIDL on TI C7x DSP\n");
  g_print ("\n");
}

/* Helper functions */
static gchar *
gst_ti_tvm_load_json_file (const gchar * file_path)
{
  GError *
      error = NULL;
  gchar *
      contents = NULL;
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
  GError *
      error = NULL;
  gchar *
      contents = NULL;

  if (!g_file_get_contents (file_path, &contents, file_size, &error)) {
    g_warning ("Failed to read parameter file %s: %s", file_path,
        error->message);
    g_error_free (error);
    return NULL;
  }

  return contents;
}
