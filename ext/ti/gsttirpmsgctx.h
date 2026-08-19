/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2026 Texas Instruments Incorporated
 *
 * Shared RPMsg channel manager for TI GStreamer plugins.
 *
 * All elements that target the same (rproc_id, remote_ep) pair
 * share a single RPMsg file descriptor.  A per-channel mutex serializes
 * send/recv pairs so concurrent elements (e.g. STFT and Deinterleave in
 * separate GStreamer threads) cannot interleave messages to the DSP.
 *
 * Lifecycle:
 *   gst_ti_rpmsg_chan_acquire() — first caller opens the channel via init_rpmsg();
 *                                 subsequent callers get a reference to the same fd.
 *   gst_ti_rpmsg_chan_release() — last caller closes the channel via cleanup_rpmsg().
 */

#ifndef __GST_TI_RPMSG_CTX_H__
#define __GST_TI_RPMSG_CTX_H__

#include <glib.h>

G_BEGIN_DECLS

typedef struct _GstTiRpmsgChan GstTiRpmsgChan;

struct _GstTiRpmsgChan {
    guint  rproc_id;    /* remote processor core ID */
    guint  remote_ep;   /* RPMsg endpoint on DSP */
    int    fd;          /* RPMsg character device file descriptor */
    gint   refcount;    /* number of elements holding this channel */
    GMutex mutex;       /* serializes one send/recv pair at a time */
};

/*
 * Returns a pointer to the shared channel, or NULL on failure.
 * Increments the reference count.  Thread-safe.
 */
GstTiRpmsgChan *gst_ti_rpmsg_chan_acquire(guint rproc_id, guint remote_ep);

/*
 * Decrements the reference count.  Closes and frees the channel when it
 * reaches zero.  Thread-safe.  Passing NULL is a no-op.
 */
void gst_ti_rpmsg_chan_release(GstTiRpmsgChan *chan);

/* Lock/unlock the channel's mutex around a send/recv pair. */
static inline void gst_ti_rpmsg_chan_lock(GstTiRpmsgChan *chan) {
    g_mutex_lock(&chan->mutex);
}

static inline void gst_ti_rpmsg_chan_unlock(GstTiRpmsgChan *chan) {
    g_mutex_unlock(&chan->mutex);
}

G_END_DECLS

#endif /* __GST_TI_RPMSG_CTX_H__ */
