/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2026 Texas Instruments Incorporated
 *
 * Generic GStreamer transform element for DSP kernel offload via TI RPMsg-DMA.
 *
 * Routes GStreamer buffers through C7x DSP kernels using zero-copy DMA buffers
 * and synchronous RPMsg IPC. Supports STFT, ISTFT, deinterleave, and interleave
 * operations with automatic state management.
 */

#ifndef __GST_DSP_KERNEL_H__
#define __GST_DSP_KERNEL_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <stdint.h>
#include "gsttirpmsgctx.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "dmabuf.h"
#ifdef __cplusplus
}
#endif

G_BEGIN_DECLS

#define GST_TYPE_DSP_KERNEL            (gst_dsp_kernel_get_type())
#define GST_DSP_KERNEL(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj),  GST_TYPE_DSP_KERNEL, GstDspKernel))
#define GST_DSP_KERNEL_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),   GST_TYPE_DSP_KERNEL, GstDspKernelClass))
#define GST_IS_DSP_KERNEL(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj),  GST_TYPE_DSP_KERNEL))
#define GST_IS_DSP_KERNEL_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),   GST_TYPE_DSP_KERNEL))

typedef struct _GstDspKernel      GstDspKernel;
typedef struct _GstDspKernelClass GstDspKernelClass;

/* DSP operation types */
typedef enum {
    DSP_OP_PASSTHROUGH    = 0x0000,  /* No operation */
    DSP_OP_STFT           = 0x1020,  /* STFT analysis (requires accumulation) */
    DSP_OP_ISTFT          = 0x1030,  /* ISTFT synthesis (requires overlap-add) */
    DSP_OP_DEINT_INTERLEAVE = 0x1040,  /* Deinterleave/Interleave (param2=flag: 0=deinterleave, 1=interleave) */
} DspOpType;

struct _GstDspKernel {
    GstBaseTransform parent;

    /* Configuration properties */
    gchar   *rproc_device;
    guint    rproc_id;
    guint    remote_ep;
    guint    msg_type;
    guint    msg_resp_type;
    guint    input_buf_size;
    guint    output_buf_size;
    guint    param0;
    guint    param1;
    guint    param2;
    guint    hop_size;          /* For STFT/ISTFT */
    guint    fft_size;          /* For STFT/ISTFT */
    guint    window_frames;     /* For STFT/ISTFT */
    guint    batch_size;        /* For STFT/ISTFT */

    /* RPMsg and DMA */
    GstTiRpmsgChan *rpmsg_chan;
    guint32         sequence_number;
    struct dma_buf_params dma_input;
    struct dma_buf_params dma_output;
    gboolean        dma_allocated;

    /* STFT-specific state (accumulation) */
    gint16  *stft_accumulator;
    gsize    stft_accumulated_samples;
    gsize    stft_total_samples;

    /* ISTFT-specific state (overlap-add) */
    gfloat  *istft_overlap_buffer;
    gsize    istft_overlap_samples;
};

struct _GstDspKernelClass {
    GstBaseTransformClass parent_class;
};

GType gst_dsp_kernel_get_type(void);

G_END_DECLS

#endif /* __GST_DSP_KERNEL_H__ */
