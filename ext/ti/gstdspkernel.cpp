/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2026 Texas Instruments Incorporated
 *
 * Generic DSP kernel transform element implementation.
 * Handles STFT, ISTFT, deinterleave, and interleave operations.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstdspkernel.h"
#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <string.h>
#include <stdint.h>

extern "C"
{
#include "rpmsg.h"
#include "dmabuf.h"
}

GST_DEBUG_CATEGORY_STATIC (gst_dsp_kernel_debug_category);
#define GST_CAT_DEFAULT gst_dsp_kernel_debug_category

/* DSP message structure */
struct c7x_msg_hdr
{
  uint32_t type;
  uint32_t seq;
  uint32_t len;
  int32_t status;
} __attribute__((packed));

struct dsp_msg
{
  struct c7x_msg_hdr hdr;
  uint32_t input_buffer;
  uint32_t output_buffer;
  uint32_t input_size;
  uint32_t output_size;
  uint32_t graph_id;
} __attribute__((packed));

#define C7X_STATUS_SUCCESS 0

/* Property IDs */
enum
{
  PROP_0,
  PROP_RPROC_DEVICE,
  PROP_RPROC_ID,
  PROP_REMOTE_EP,
  PROP_MSG_TYPE,
  PROP_MSG_RESP_TYPE,
  PROP_INPUT_BUF_SIZE,
  PROP_OUTPUT_BUF_SIZE,
  PROP_PARAM0,
  PROP_PARAM1,
  PROP_PARAM2,
  PROP_HOP_SIZE,
  PROP_FFT_SIZE,
  PROP_WINDOW_FRAMES,
  PROP_BATCH_SIZE,
};

/* Defaults */
#define DEFAULT_RPROC_DEVICE    "/dev/remoteproc0"
#define DEFAULT_RPROC_ID        8
#define DEFAULT_REMOTE_EP       13
#define DEFAULT_MSG_TYPE        0
#define DEFAULT_MSG_RESP_TYPE   0
#define DEFAULT_INPUT_BUF_SIZE  (64 * 1024)
#define DEFAULT_OUTPUT_BUF_SIZE (256 * 1024)
#define DEFAULT_PARAM0          0
#define DEFAULT_PARAM1          0
#define DEFAULT_PARAM2          0
#define DEFAULT_HOP_SIZE        160
#define DEFAULT_FFT_SIZE        320
#define DEFAULT_WINDOW_FRAMES   401
#define DEFAULT_BATCH_SIZE      64

/* Function prototypes */
static void gst_dsp_kernel_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_dsp_kernel_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_dsp_kernel_finalize (GObject * object);
static gboolean gst_dsp_kernel_start (GstBaseTransform * trans);
static gboolean gst_dsp_kernel_stop (GstBaseTransform * trans);
static GstFlowReturn gst_dsp_kernel_transform_ip (GstBaseTransform * trans,
    GstBuffer * buf);

static GstStaticPadTemplate dsp_kernel_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS ("application/octet-stream"));
static GstStaticPadTemplate dsp_kernel_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS ("application/octet-stream"));

#define gst_dsp_kernel_parent_class parent_class
G_DEFINE_TYPE_WITH_CODE (GstDspKernel, gst_dsp_kernel, GST_TYPE_BASE_TRANSFORM,
    GST_DEBUG_CATEGORY_INIT (gst_dsp_kernel_debug_category, "tidspkernel", 0,
        "TI DSP Kernel Transform"));

static void
gst_dsp_kernel_class_init (GstDspKernelClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *bt = GST_BASE_TRANSFORM_CLASS (klass);

  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_static_pad_template_get (&dsp_kernel_sink_template));
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_static_pad_template_get (&dsp_kernel_src_template));
  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "TI DSP Kernel Transform", "Transform/DSP",
      "Generic DSP kernel offload (STFT/ISTFT/Deinterleave/Interleave) via RPMsg-DMA",
      "Pratham Deshmukh <p-deshmukh@ti.com>");

  gobject_class->set_property = gst_dsp_kernel_set_property;
  gobject_class->get_property = gst_dsp_kernel_get_property;
  gobject_class->finalize = gst_dsp_kernel_finalize;

  bt->start = GST_DEBUG_FUNCPTR (gst_dsp_kernel_start);
  bt->stop = GST_DEBUG_FUNCPTR (gst_dsp_kernel_stop);
  bt->transform_ip = GST_DEBUG_FUNCPTR (gst_dsp_kernel_transform_ip);
  bt->passthrough_on_same_caps = FALSE;

  /* Install properties */
  g_object_class_install_property (gobject_class, PROP_RPROC_DEVICE,
      g_param_spec_string ("rproc-device", "Remoteproc Device",
          "Remoteproc cdev for DMA buffer physical address lookup",
          DEFAULT_RPROC_DEVICE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_RPROC_ID,
      g_param_spec_uint ("rproc-id", "Remote Processor ID",
          "Linux remoteproc core ID (8 = C7x_0 on AM62D)",
          0, G_MAXUINT, DEFAULT_RPROC_ID,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_REMOTE_EP,
      g_param_spec_uint ("remote-ep", "Remote RPMsg Endpoint",
          "RPMsg endpoint number running DSP firmware",
          0, G_MAXUINT, DEFAULT_REMOTE_EP,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_MSG_TYPE,
      g_param_spec_uint ("msg-type", "DSP Message Type",
          "IPC message type (0x1020=STFT, 0x1030=ISTFT, 0x1040=De/Interleave)",
          0, G_MAXUINT, DEFAULT_MSG_TYPE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_MSG_RESP_TYPE,
      g_param_spec_uint ("msg-resp-type", "DSP Response Type",
          "Expected response opcode (0 = auto-derive)",
          0, G_MAXUINT, DEFAULT_MSG_RESP_TYPE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_INPUT_BUF_SIZE,
      g_param_spec_uint ("input-buf-size", "Input Buffer Size",
          "DMA input buffer size in bytes",
          1, G_MAXUINT, DEFAULT_INPUT_BUF_SIZE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_OUTPUT_BUF_SIZE,
      g_param_spec_uint ("output-buf-size", "Output Buffer Size",
          "DMA output buffer size in bytes",
          0, G_MAXUINT, DEFAULT_OUTPUT_BUF_SIZE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_PARAM0,
      g_param_spec_uint ("param0", "Parameter 0",
          "DSP kernel parameter 0 (input_frames for STFT/ISTFT)",
          0, G_MAXUINT, DEFAULT_PARAM0,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_PARAM1,
      g_param_spec_uint ("param1", "Parameter 1",
          "DSP kernel parameter 1 (output_frames/fft_size)",
          0, G_MAXUINT, DEFAULT_PARAM1,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_PARAM2,
      g_param_spec_uint ("param2", "Parameter 2",
          "DSP kernel parameter 2 (graph_id/flag: 0=deinterleave, 1=interleave)",
          0, G_MAXUINT, DEFAULT_PARAM2,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_HOP_SIZE,
      g_param_spec_uint ("hop-size", "Hop Size",
          "Hop size between frames (for STFT/ISTFT)",
          1, 8192, DEFAULT_HOP_SIZE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_FFT_SIZE,
      g_param_spec_uint ("fft-size", "FFT Size",
          "FFT size in samples (for STFT/ISTFT)",
          1, 8192, DEFAULT_FFT_SIZE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_WINDOW_FRAMES,
      g_param_spec_uint ("window-frames", "Window Frames",
          "Total frames to accumulate (for STFT/ISTFT)",
          1, 8192, DEFAULT_WINDOW_FRAMES,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_BATCH_SIZE,
      g_param_spec_uint ("batch-size", "Batch Size",
          "Frames per batch (for STFT/ISTFT)",
          1, 8192, DEFAULT_BATCH_SIZE,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}

static void
gst_dsp_kernel_init (GstDspKernel * kernel)
{
  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (kernel), TRUE);

  kernel->rproc_device = g_strdup (DEFAULT_RPROC_DEVICE);
  kernel->rproc_id = DEFAULT_RPROC_ID;
  kernel->remote_ep = DEFAULT_REMOTE_EP;
  kernel->msg_type = DEFAULT_MSG_TYPE;
  kernel->msg_resp_type = DEFAULT_MSG_RESP_TYPE;
  kernel->input_buf_size = DEFAULT_INPUT_BUF_SIZE;
  kernel->output_buf_size = DEFAULT_OUTPUT_BUF_SIZE;
  kernel->param0 = DEFAULT_PARAM0;
  kernel->param1 = DEFAULT_PARAM1;
  kernel->param2 = DEFAULT_PARAM2;
  kernel->hop_size = DEFAULT_HOP_SIZE;
  kernel->fft_size = DEFAULT_FFT_SIZE;
  kernel->window_frames = DEFAULT_WINDOW_FRAMES;
  kernel->batch_size = DEFAULT_BATCH_SIZE;

  kernel->rpmsg_chan = NULL;
  kernel->sequence_number = 1;
  kernel->dma_allocated = FALSE;

  kernel->stft_accumulator = NULL;
  kernel->stft_accumulated_samples = 0;
  kernel->stft_total_samples = 0;

  kernel->istft_overlap_buffer = NULL;
  kernel->istft_overlap_samples = 0;

  memset (&kernel->dma_input, 0, sizeof (kernel->dma_input));
  memset (&kernel->dma_output, 0, sizeof (kernel->dma_output));
}

static void
gst_dsp_kernel_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstDspKernel *kernel = GST_DSP_KERNEL (object);
  GST_OBJECT_LOCK (kernel);
  switch (prop_id) {
    case PROP_RPROC_DEVICE:
      g_free (kernel->rproc_device);
      kernel->rproc_device = g_value_dup_string (value);
      break;
    case PROP_RPROC_ID:
      kernel->rproc_id = g_value_get_uint (value);
      break;
    case PROP_REMOTE_EP:
      kernel->remote_ep = g_value_get_uint (value);
      break;
    case PROP_MSG_TYPE:
      kernel->msg_type = g_value_get_uint (value);
      break;
    case PROP_MSG_RESP_TYPE:
      kernel->msg_resp_type = g_value_get_uint (value);
      break;
    case PROP_INPUT_BUF_SIZE:
      kernel->input_buf_size = g_value_get_uint (value);
      break;
    case PROP_OUTPUT_BUF_SIZE:
      kernel->output_buf_size = g_value_get_uint (value);
      break;
    case PROP_PARAM0:
      kernel->param0 = g_value_get_uint (value);
      break;
    case PROP_PARAM1:
      kernel->param1 = g_value_get_uint (value);
      break;
    case PROP_PARAM2:
      kernel->param2 = g_value_get_uint (value);
      break;
    case PROP_HOP_SIZE:
      kernel->hop_size = g_value_get_uint (value);
      break;
    case PROP_FFT_SIZE:
      kernel->fft_size = g_value_get_uint (value);
      break;
    case PROP_WINDOW_FRAMES:
      kernel->window_frames = g_value_get_uint (value);
      break;
    case PROP_BATCH_SIZE:
      kernel->batch_size = g_value_get_uint (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
  }
  GST_OBJECT_UNLOCK (kernel);
}

static void
gst_dsp_kernel_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstDspKernel *kernel = GST_DSP_KERNEL (object);
  GST_OBJECT_LOCK (kernel);
  switch (prop_id) {
    case PROP_RPROC_DEVICE:
      g_value_set_string (value, kernel->rproc_device);
      break;
    case PROP_RPROC_ID:
      g_value_set_uint (value, kernel->rproc_id);
      break;
    case PROP_REMOTE_EP:
      g_value_set_uint (value, kernel->remote_ep);
      break;
    case PROP_MSG_TYPE:
      g_value_set_uint (value, kernel->msg_type);
      break;
    case PROP_MSG_RESP_TYPE:
      g_value_set_uint (value, kernel->msg_resp_type);
      break;
    case PROP_INPUT_BUF_SIZE:
      g_value_set_uint (value, kernel->input_buf_size);
      break;
    case PROP_OUTPUT_BUF_SIZE:
      g_value_set_uint (value, kernel->output_buf_size);
      break;
    case PROP_PARAM0:
      g_value_set_uint (value, kernel->param0);
      break;
    case PROP_PARAM1:
      g_value_set_uint (value, kernel->param1);
      break;
    case PROP_PARAM2:
      g_value_set_uint (value, kernel->param2);
      break;
    case PROP_HOP_SIZE:
      g_value_set_uint (value, kernel->hop_size);
      break;
    case PROP_FFT_SIZE:
      g_value_set_uint (value, kernel->fft_size);
      break;
    case PROP_WINDOW_FRAMES:
      g_value_set_uint (value, kernel->window_frames);
      break;
    case PROP_BATCH_SIZE:
      g_value_set_uint (value, kernel->batch_size);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
  }
  GST_OBJECT_UNLOCK (kernel);
}

static void
gst_dsp_kernel_finalize (GObject * object)
{
  GstDspKernel *kernel = GST_DSP_KERNEL (object);
  g_free (kernel->rproc_device);
  if (kernel->stft_accumulator) {
    g_free (kernel->stft_accumulator);
    kernel->stft_accumulator = NULL;
  }
  if (kernel->istft_overlap_buffer) {
    g_free (kernel->istft_overlap_buffer);
    kernel->istft_overlap_buffer = NULL;
  }
  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static gboolean
gst_dsp_kernel_start (GstBaseTransform * trans)
{
  GstDspKernel *kernel = GST_DSP_KERNEL (trans);

  g_print ("TI DSP Kernel Element\n");
  g_print ("=====================\n");
  g_print ("[DSP] msg-type: 0x%04x ", kernel->msg_type);
  switch (kernel->msg_type) {
    case DSP_OP_STFT:
      g_print ("(STFT)\n");
      kernel->stft_total_samples = kernel->hop_size * kernel->window_frames;
      kernel->stft_accumulator = g_new0 (gint16, kernel->stft_total_samples);
      kernel->stft_accumulated_samples = 0;
      g_print ("[DSP] STFT: accumulating %zu samples (%.2f sec @ 16kHz)\n",
          kernel->stft_total_samples, kernel->stft_total_samples / 16000.0);
      break;
    case DSP_OP_ISTFT:
      g_print ("(ISTFT)\n");
      kernel->istft_overlap_samples = kernel->fft_size - kernel->hop_size;
      kernel->istft_overlap_buffer = g_new0 (gfloat, kernel->fft_size);
      g_print ("[DSP] ISTFT: overlap-add with %zu samples\n",
          kernel->istft_overlap_samples);
      break;
    case DSP_OP_DEINT_INTERLEAVE:
      g_print ("(Deinterleave/Interleave, flag=%u)\n", kernel->param2);
      break;
    default:
      g_print ("(Unknown)\n");
      break;
  }

  /* Acquire shared RPMsg channel */
  kernel->rpmsg_chan =
      gst_ti_rpmsg_chan_acquire (kernel->rproc_id, kernel->remote_ep);
  if (!kernel->rpmsg_chan) {
    GST_ERROR_OBJECT (kernel, "Failed to acquire rpmsg channel");
    return FALSE;
  }
  g_print ("[DSP] RPMsg channel acquired (fd=%d)\n", kernel->rpmsg_chan->fd);

  /* Allocate DMA buffers */
  int r1 = dmabuf_heap_init ((char *) "linux,cma", kernel->input_buf_size,
      kernel->rproc_device, &kernel->dma_input);
  int r2 = dmabuf_heap_init ((char *) "linux,cma", kernel->output_buf_size,
      kernel->rproc_device, &kernel->dma_output);

  if (r1 != 0 || r2 != 0) {
    GST_ERROR_OBJECT (kernel, "DMA alloc failed (r1=%d r2=%d)", r1, r2);
    if (r1 == 0)
      dmabuf_heap_destroy (&kernel->dma_input);
    gst_ti_rpmsg_chan_release (kernel->rpmsg_chan);
    kernel->rpmsg_chan = NULL;
    return FALSE;
  }

  kernel->dma_allocated = TRUE;
  g_print ("[DSP] DMA buffers allocated\n");
  g_print ("[DSP]   input:  %u bytes @ phys=0x%08lx\n",
      kernel->input_buf_size, (unsigned long) kernel->dma_input.phys_addr);
  g_print ("[DSP]   output: %u bytes @ phys=0x%08lx\n",
      kernel->output_buf_size, (unsigned long) kernel->dma_output.phys_addr);

  return TRUE;
}

static gboolean
gst_dsp_kernel_stop (GstBaseTransform * trans)
{
  GstDspKernel *kernel = GST_DSP_KERNEL (trans);

  if (kernel->dma_allocated) {
    dmabuf_heap_destroy (&kernel->dma_input);
    dmabuf_heap_destroy (&kernel->dma_output);
    kernel->dma_allocated = FALSE;
  }

  if (kernel->stft_accumulator) {
    g_free (kernel->stft_accumulator);
    kernel->stft_accumulator = NULL;
  }
  kernel->stft_accumulated_samples = 0;

  if (kernel->istft_overlap_buffer) {
    g_free (kernel->istft_overlap_buffer);
    kernel->istft_overlap_buffer = NULL;
  }

  gst_ti_rpmsg_chan_release (kernel->rpmsg_chan);
  kernel->rpmsg_chan = NULL;

  return TRUE;
}

/* Helper: Send DSP message and receive response */
static GstFlowReturn
dsp_kernel_send_recv (GstDspKernel * kernel, struct dsp_msg *req,
    struct dsp_msg *resp)
{
  uint32_t expected_resp = kernel->msg_resp_type ?
      kernel->msg_resp_type : ((kernel->msg_type & 0x0FFF) | 0x2000);

  gst_ti_rpmsg_chan_lock (kernel->rpmsg_chan);

  if (send_msg (kernel->rpmsg_chan->fd, (char *) req, sizeof (*req)) < 0) {
    GST_ERROR_OBJECT (kernel, "send_msg failed");
    gst_ti_rpmsg_chan_unlock (kernel->rpmsg_chan);
    return GST_FLOW_ERROR;
  }

  int resp_len = sizeof (*resp);
  if (recv_msg (kernel->rpmsg_chan->fd, sizeof (*resp), (char *) resp,
          &resp_len) < 0) {
    GST_ERROR_OBJECT (kernel, "recv_msg failed");
    gst_ti_rpmsg_chan_unlock (kernel->rpmsg_chan);
    return GST_FLOW_ERROR;
  }

  gst_ti_rpmsg_chan_unlock (kernel->rpmsg_chan);

  if (resp->hdr.type != expected_resp || resp->hdr.status != C7X_STATUS_SUCCESS) {
    GST_ERROR_OBJECT (kernel, "DSP error: type=0x%x status=%d",
        resp->hdr.type, resp->hdr.status);
    return GST_FLOW_ERROR;
  }

  return GST_FLOW_OK;
}

/* STFT transform with accumulation */
static GstFlowReturn
dsp_kernel_transform_stft (GstDspKernel * kernel, GstBuffer * buf)
{
  GstMapInfo map_info;
  if (!gst_buffer_map (buf, &map_info, GST_MAP_READWRITE)) {
    GST_ERROR_OBJECT (kernel, "Failed to map buffer");
    return GST_FLOW_ERROR;
  }

  const gint16 *audio_in = (const gint16 *) map_info.data;
  gsize incoming_samples = map_info.size / sizeof (gint16);

  /* Accumulate samples */
  gsize samples_to_copy = MIN (incoming_samples,
      kernel->stft_total_samples - kernel->stft_accumulated_samples);
  memcpy (kernel->stft_accumulator + kernel->stft_accumulated_samples,
      audio_in, samples_to_copy * sizeof (gint16));
  kernel->stft_accumulated_samples += samples_to_copy;

  /* If not enough samples yet, return empty buffer */
  if (kernel->stft_accumulated_samples < kernel->stft_total_samples) {
    g_print ("[STFT] Accumulating... %zu/%zu samples (%.1f%%)\n",
        kernel->stft_accumulated_samples, kernel->stft_total_samples,
        100.0 * kernel->stft_accumulated_samples / kernel->stft_total_samples);
    gst_buffer_set_size (buf, 0);
    gst_buffer_unmap (buf, &map_info);
    return GST_FLOW_OK;
  }

  g_print ("[STFT] Processing %zu samples in batches...\n",
      kernel->stft_total_samples);

  /* Calculate batches */
  guint num_batches =
      (kernel->window_frames + kernel->batch_size - 1) / kernel->batch_size;
  gsize out_buf_size = num_batches * kernel->output_buf_size;
  guint8 *spectral_out = (guint8 *) g_malloc0 (out_buf_size);
  gsize out_written = 0;

  /* Process in batches */
  for (guint batch_idx = 0; batch_idx < num_batches; batch_idx++) {
    guint frame_start = batch_idx * kernel->batch_size;
    guint frames_in_batch =
        MIN (kernel->batch_size, kernel->window_frames - frame_start);
    gsize batch_samples = frames_in_batch * kernel->hop_size;
    gsize batch_bytes = batch_samples * sizeof (gint16);
    gsize sample_offset = frame_start * kernel->hop_size;

    /* Copy batch to DMA input */
    dmabuf_sync (kernel->dma_input.dma_buf_fd, DMA_BUF_SYNC_START);
    memcpy (kernel->dma_input.kern_addr,
        kernel->stft_accumulator + sample_offset, batch_bytes);
    dmabuf_sync (kernel->dma_input.dma_buf_fd, DMA_BUF_SYNC_END);

    /* Send to DSP */
    struct dsp_msg req = { }, resp = { };
    req.hdr.type = kernel->msg_type;
    req.hdr.seq = kernel->sequence_number++;
    req.hdr.len = sizeof (req);
    req.input_buffer = (uint32_t) kernel->dma_input.phys_addr;
    req.output_buffer = (uint32_t) kernel->dma_output.phys_addr;
    req.input_size = (uint32_t) batch_bytes;
    req.output_size = kernel->output_buf_size;
    req.graph_id = kernel->param2;

    if (dsp_kernel_send_recv (kernel, &req, &resp) != GST_FLOW_OK) {
      g_free (spectral_out);
      gst_buffer_unmap (buf, &map_info);
      return GST_FLOW_ERROR;
    }

    /* Copy DSP output */
    gsize batch_spectral_bytes = resp.output_size;
    dmabuf_sync (kernel->dma_output.dma_buf_fd, DMA_BUF_SYNC_START);
    memcpy (spectral_out + out_written, kernel->dma_output.kern_addr,
        batch_spectral_bytes);
    dmabuf_sync (kernel->dma_output.dma_buf_fd, DMA_BUF_SYNC_END);

    out_written += batch_spectral_bytes;
  }

  g_print ("[STFT] Done: %zu samples → %zu spectral bytes\n",
      kernel->stft_total_samples, out_written);

  /* Reset accumulator */
  kernel->stft_accumulated_samples = 0;

  /* Replace buffer contents */
  memcpy ((void *) map_info.data, spectral_out, out_written);
  gst_buffer_set_size (buf, (gssize) out_written);

  g_free (spectral_out);
  gst_buffer_unmap (buf, &map_info);
  return GST_FLOW_OK;
}

/* ISTFT transform with overlap-add */
static GstFlowReturn
dsp_kernel_transform_istft (GstDspKernel * kernel, GstBuffer * buf)
{
  GstMapInfo map_info;
  if (!gst_buffer_map (buf, &map_info, GST_MAP_READWRITE)) {
    GST_ERROR_OBJECT (kernel, "Failed to map buffer");
    return GST_FLOW_ERROR;
  }

  const guint8 *spectral_in = (const guint8 *) map_info.data;
  gsize total_spectral_bytes = map_info.size;

  g_print ("[ISTFT] Processing %zu spectral bytes...\n", total_spectral_bytes);

  /* Calculate batches */
  guint num_batches =
      (kernel->window_frames + kernel->batch_size - 1) / kernel->batch_size;
  gsize max_output_samples = kernel->window_frames * kernel->hop_size;
  gint16 *audio_out =
      (gint16 *) g_malloc0 (max_output_samples * sizeof (gint16));
  gsize out_written_samples = 0;
  gsize spectral_offset = 0;

  /* Process in batches */
  for (guint batch_idx = 0; batch_idx < num_batches; batch_idx++) {
    guint frame_start = batch_idx * kernel->batch_size;
    guint frames_in_batch =
        MIN (kernel->batch_size, kernel->window_frames - frame_start);
    gsize batch_spectral_bytes = frames_in_batch * 322 * sizeof (float);

    if (spectral_offset + batch_spectral_bytes > total_spectral_bytes) {
      batch_spectral_bytes = total_spectral_bytes - spectral_offset;
    }

    /* Copy spectral batch to DMA input */
    dmabuf_sync (kernel->dma_input.dma_buf_fd, DMA_BUF_SYNC_START);
    memcpy (kernel->dma_input.kern_addr, spectral_in + spectral_offset,
        batch_spectral_bytes);
    dmabuf_sync (kernel->dma_input.dma_buf_fd, DMA_BUF_SYNC_END);

    /* Send to DSP */
    struct dsp_msg req = { }, resp = { };
    req.hdr.type = kernel->msg_type;
    req.hdr.seq = kernel->sequence_number++;
    req.hdr.len = sizeof (req);
    req.input_buffer = (uint32_t) kernel->dma_input.phys_addr;
    req.output_buffer = (uint32_t) kernel->dma_output.phys_addr;
    req.input_size = (uint32_t) batch_spectral_bytes;
    req.output_size = kernel->output_buf_size;
    req.graph_id = kernel->param2;

    if (dsp_kernel_send_recv (kernel, &req, &resp) != GST_FLOW_OK) {
      g_free (audio_out);
      gst_buffer_unmap (buf, &map_info);
      return GST_FLOW_ERROR;
    }

    /* Copy DSP output */
    gsize batch_audio_bytes = resp.output_size;
    gsize batch_audio_samples = batch_audio_bytes / sizeof (gint16);
    dmabuf_sync (kernel->dma_output.dma_buf_fd, DMA_BUF_SYNC_START);
    memcpy (audio_out + out_written_samples, kernel->dma_output.kern_addr,
        batch_audio_bytes);
    dmabuf_sync (kernel->dma_output.dma_buf_fd, DMA_BUF_SYNC_END);

    out_written_samples += batch_audio_samples;
    spectral_offset += batch_spectral_bytes;
  }

  g_print ("[ISTFT] Done: %zu spectral bytes → %zu audio samples\n",
      total_spectral_bytes, out_written_samples);

  /* Replace buffer contents */
  gsize output_bytes = out_written_samples * sizeof (gint16);
  memcpy ((void *) map_info.data, audio_out, output_bytes);
  gst_buffer_set_size (buf, (gssize) output_bytes);

  g_free (audio_out);
  gst_buffer_unmap (buf, &map_info);
  return GST_FLOW_OK;
}

/* Simple pass-through transform (deinterleave/interleave) */
static GstFlowReturn
dsp_kernel_transform_passthrough (GstDspKernel * kernel, GstBuffer * buf)
{
  GstMapInfo map_info;
  if (!gst_buffer_map (buf, &map_info, GST_MAP_READWRITE)) {
    GST_ERROR_OBJECT (kernel, "Failed to map buffer");
    return GST_FLOW_ERROR;
  }

  gsize input_size = map_info.size;

  /* Copy to DMA input */
  dmabuf_sync (kernel->dma_input.dma_buf_fd, DMA_BUF_SYNC_START);
  memcpy (kernel->dma_input.kern_addr, map_info.data, input_size);
  dmabuf_sync (kernel->dma_input.dma_buf_fd, DMA_BUF_SYNC_END);

  /* Send to DSP */
  struct dsp_msg req = { }, resp = { };
  req.hdr.type = kernel->msg_type;
  req.hdr.seq = kernel->sequence_number++;
  req.hdr.len = sizeof (req);
  req.input_buffer = (uint32_t) kernel->dma_input.phys_addr;
  req.output_buffer = (uint32_t) kernel->dma_output.phys_addr;
  req.input_size = (uint32_t) input_size;
  req.output_size = kernel->output_buf_size;
  req.graph_id = kernel->param2;

  if (dsp_kernel_send_recv (kernel, &req, &resp) != GST_FLOW_OK) {
    gst_buffer_unmap (buf, &map_info);
    return GST_FLOW_ERROR;
  }

  /* Copy DSP output */
  gsize output_size = resp.output_size;
  dmabuf_sync (kernel->dma_output.dma_buf_fd, DMA_BUF_SYNC_START);
  memcpy ((void *) map_info.data, kernel->dma_output.kern_addr, output_size);
  dmabuf_sync (kernel->dma_output.dma_buf_fd, DMA_BUF_SYNC_END);

  gst_buffer_set_size (buf, (gssize) output_size);
  gst_buffer_unmap (buf, &map_info);
  return GST_FLOW_OK;
}

static GstFlowReturn
gst_dsp_kernel_transform_ip (GstBaseTransform * trans, GstBuffer * buf)
{
  GstDspKernel *kernel = GST_DSP_KERNEL (trans);

  switch (kernel->msg_type) {
    case DSP_OP_STFT:
      return dsp_kernel_transform_stft (kernel, buf);
    case DSP_OP_ISTFT:
      return dsp_kernel_transform_istft (kernel, buf);
    case DSP_OP_DEINT_INTERLEAVE:
    default:
      return dsp_kernel_transform_passthrough (kernel, buf);
  }
}
