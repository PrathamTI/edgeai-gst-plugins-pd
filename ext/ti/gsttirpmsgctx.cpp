/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2026 Texas Instruments Incorporated
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gsttirpmsgctx.h"
#include <gst/gst.h>

extern "C"
{
#include "rpmsg.h"
}

GST_DEBUG_CATEGORY_STATIC (gst_ti_rpmsg_chan_debug);
#define GST_CAT_DEFAULT gst_ti_rpmsg_chan_debug

/* Process-wide registry of open RPMsg channels, guarded by s_lock. */
static GMutex s_lock;
static GList *s_channels = NULL;        /* list of GstTiRpmsgChan* */
static gboolean s_debug_initialized = FALSE;

GstTiRpmsgChan *
gst_ti_rpmsg_chan_acquire (guint rproc_id, guint remote_ep)
{
  GstTiRpmsgChan *chan = NULL;

  g_mutex_lock (&s_lock);

  /* Initialize debug category once */
  if (!s_debug_initialized) {
    GST_DEBUG_CATEGORY_INIT (gst_ti_rpmsg_chan_debug, "tirpmsgchan", 0,
        "TI RPMsg Channel Manager");
    s_debug_initialized = TRUE;
  }

  /* Re-use an existing open channel for the same DSP endpoint. */
  for (GList * l = s_channels; l; l = l->next) {
    GstTiRpmsgChan *c = (GstTiRpmsgChan *) l->data;
    if (c->rproc_id == rproc_id && c->remote_ep == remote_ep) {
      g_atomic_int_inc (&c->refcount);
      GST_INFO ("Reusing RPMsg channel (rproc=%u ep=%u fd=%d) refcount=%d",
          rproc_id, remote_ep, c->fd, g_atomic_int_get (&c->refcount));
      chan = c;
      goto done;
    }
  }

  /* No existing channel — open a new one. */
  {
    int fd = init_rpmsg ((int) rproc_id, (int) remote_ep);
    if (fd < 0) {
      GST_ERROR
          ("init_rpmsg failed (rproc=%u ep=%u) — is DSP firmware loaded?",
          rproc_id, remote_ep);
      goto done;
    }

    chan = g_new0 (GstTiRpmsgChan, 1);
    chan->rproc_id = rproc_id;
    chan->remote_ep = remote_ep;
    chan->fd = fd;
    chan->refcount = 1;
    g_mutex_init (&chan->mutex);

    s_channels = g_list_prepend (s_channels, chan);
    GST_INFO ("Opened RPMsg channel (rproc=%u ep=%u fd=%d)",
        rproc_id, remote_ep, fd);
  }

done:
  g_mutex_unlock (&s_lock);
  return chan;
}

void
gst_ti_rpmsg_chan_release (GstTiRpmsgChan * chan)
{
  if (!chan)
    return;

  g_mutex_lock (&s_lock);

  if (g_atomic_int_dec_and_test (&chan->refcount)) {
    GST_INFO ("Closing RPMsg channel (rproc=%u ep=%u fd=%d)",
        chan->rproc_id, chan->remote_ep, chan->fd);
    s_channels = g_list_remove (s_channels, chan);
    cleanup_rpmsg (chan->fd);
    g_mutex_clear (&chan->mutex);
    g_free (chan);
  } else {
    GST_INFO ("Released RPMsg channel ref (rproc=%u ep=%u) refcount=%d",
        chan->rproc_id, chan->remote_ep, g_atomic_int_get (&chan->refcount));
  }

  g_mutex_unlock (&s_lock);
}
