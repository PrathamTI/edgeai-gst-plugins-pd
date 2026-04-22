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
 * *    No reverse engineering, decompilation, or disassembly of this software
 *      is permitted with respect to any software provided in binary form.
 *
 * *    Any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *    Nothing shall obligate TI to provide you with source code for the
 *      software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution
 * of the source code are permitted provided that the following conditions are met:
 *
 * *    Any redistribution and use of the source code, including any resulting
 *      derivative works, are licensed by TI for use only with TI Devices.
 *
 * *    Any redistribution and use of any object code compiled from the source
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
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gst/check/gstcheck.h>
#include "gst-libs/gst/tiovx/gsttiovxutils.h"
#include "test_utils.h"

GST_DEBUG_CATEGORY_STATIC (gst_tiovx_fc_test_debug);

#undef GST_CAT_DEFAULT
#define GST_CAT_DEFAULT gst_tiovx_fc_test_debug

#define TIOVXFC_STATE_CHANGE_ITERATIONS 1

/* Default number of exposures */
static const gint default_num_exposures = 1;

typedef struct
{
  const guint min;
  const guint max;
} Range;

#define TIOVXFC_INPUT_FORMATS_ARRAY_SIZE 4
static const gchar *tiovxfc_input_formats[TIOVXFC_INPUT_FORMATS_ARRAY_SIZE] = {
  "bggr",
  "gbrg",
  "grbg",
  "rggb",
};

#define TIOVXFC_OUTPUT_FORMATS_ARRAY_SIZE 2
static const gchar *tiovxfc_output_formats[TIOVXFC_OUTPUT_FORMATS_ARRAY_SIZE] = {
  "NV12",
  "GRAY8",
};

static const Range tiovxfc_width = { 128, 2048 };
static const Range tiovxfc_height = { 128, 1080 };

static const Range tiovxfc_pool_size = { 2, 16 };

#define TIOVXFC_TARGET_ARRAY_SIZE 1
static const gchar *tiovxfc_target[TIOVXFC_TARGET_ARRAY_SIZE] = {
  "VPAC_FC",
};

typedef struct
{
  const Range *width_range;
  const Range *height_range;
  const gchar **formats;
  const Range *pool_size_range;
} PadTemplate;

typedef struct
{
  const gchar **target;
} Properties;

typedef struct
{
  PadTemplate src_pad;
  PadTemplate sink_pad;
  Properties properties;
} TIOVXFCModeled;

static const void gst_tiovx_fc_modeling_init (TIOVXFCModeled * element);

static const void
gst_tiovx_fc_modeling_init (TIOVXFCModeled * element)
{
  element->sink_pad.formats = tiovxfc_input_formats;
  element->sink_pad.width_range = &tiovxfc_width;
  element->sink_pad.height_range = &tiovxfc_height;
  element->sink_pad.pool_size_range = &tiovxfc_pool_size;

  element->src_pad.formats = tiovxfc_output_formats;
  element->src_pad.width_range = &tiovxfc_width;
  element->src_pad.height_range = &tiovxfc_height;
  element->src_pad.pool_size_range = &tiovxfc_pool_size;

  element->properties.target = tiovxfc_target;
}

static inline const guint
gst_tiovx_fc_get_blocksize (const guint width,
    const guint height, const gchar * formats, const gint num_exposures)
{
  guint bits_per_pixel = 0;
  guint blocksize = 0;

  if (G_UNLIKELY (NULL == formats)) {
    return 0;
  }

  bits_per_pixel = gst_tiovx_bayer_get_bits_per_pixel (formats);
  blocksize = width * height * bits_per_pixel * num_exposures;

  return blocksize;
}

static inline const gint
gst_tiovx_fc_get_int_range_pair_value (gint begin, gint end)
{
  gint value = 0;
  gint attempt = 0;

  if (G_UNLIKELY (begin >= end)) {
    return begin;
  }

  do {
    value = g_random_int_range (begin, end);
    attempt++;
  } while ((1 == value % 2));

  return value;
}

GST_START_TEST (test_output_format_fail)
{
  TIOVXFCModeled element = { 0 };
  g_autoptr (GString) pipeline = g_string_new ("");
  g_autoptr (GString) sink_caps = g_string_new ("");
  g_autoptr (GString) sink_src = g_string_new ("");
  g_autoptr (GString) src_caps = g_string_new ("");
  guint width = 0;
  guint height = 0;
  const gchar *format = NULL;
  guint blocksize = 0;
  guint i = 0;

  gst_tiovx_fc_modeling_init (&element);

  width =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.width_range->min,
      element.sink_pad.width_range->max);
  height =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.height_range->
      min, element.sink_pad.height_range->max);

  for (i = 0; i < TIOVXFC_INPUT_FORMATS_ARRAY_SIZE; i++)
  {
    format = element.sink_pad.formats[i];
    blocksize =
        gst_tiovx_fc_get_blocksize (width, height, format,
        default_num_exposures);

    g_string_printf (sink_caps, "video/x-bayer,width=%d,height=%d,format=%s",
        width, height, format);

    g_string_printf (sink_src, "filesrc location=/dev/zero blocksize=%d num-buffers=10 ! %s",
        blocksize, sink_caps->str);

    /* Src - invalid output format */
    g_string_printf (src_caps, "video/x-video,format=%d",
        GST_VIDEO_FORMAT_UNKNOWN);

    g_string_printf (pipeline,
        "%s ! tiovxfcvissmsc target=%s ! %s ! fakesink",
        sink_src->str, element.properties.target[0], src_caps->str);

    test_create_pipeline_fail (pipeline->str);
    }

}
GST_END_TEST

GST_START_TEST (test_input_format_fail)
{
  TIOVXFCModeled element = { 0 };
  g_autoptr (GString) pipeline = g_string_new ("");
  g_autoptr (GString) sink_caps = g_string_new ("");
  g_autoptr (GString) sink_src = g_string_new ("");
  guint width = 0;
  guint height = 0;

  gst_tiovx_fc_modeling_init (&element);

  width =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.width_range->min,
      element.sink_pad.width_range->max);
  height =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.height_range->
      min, element.sink_pad.height_range->max);

  g_string_printf (sink_caps, "video/x-bayer,width=%d,height=%d,format=%d",
      width, height, GST_VIDEO_FORMAT_UNKNOWN);

  g_string_printf (sink_src, "filesrc location=/dev/zero ! %s", sink_caps->str);

  g_string_printf (pipeline,
      "%s ! tiovxfcvissmsc target=%s ! fakesink",
      sink_src->str, element.properties.target[0]);

  test_states_change_async_fail_success (pipeline->str, TIOVXFC_STATE_CHANGE_ITERATIONS);

}
GST_END_TEST

GST_START_TEST (test_foreach_format)
{
  TIOVXFCModeled element = { 0 };
  g_autoptr (GString) pipeline = g_string_new ("");
  g_autoptr (GString) sink_caps = g_string_new ("");
  g_autoptr (GString) sink_src = g_string_new ("");
  g_autoptr (GString) src_caps = g_string_new ("");
  guint width = 0;
  guint height = 0;
  guint blocksize = 0;
  guint i = 0;

  gst_tiovx_fc_modeling_init (&element);

  width =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.width_range->min,
      element.sink_pad.width_range->max);
  height =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.
      height_range->min, element.sink_pad.height_range->max);

  g_string_printf (src_caps, "video/x-raw,format=NV12");

  for (i = 0; i < TIOVXFC_INPUT_FORMATS_ARRAY_SIZE; i++) {

    blocksize =
        gst_tiovx_fc_get_blocksize (width, height, element.sink_pad.formats[i],
        default_num_exposures);

    g_string_printf (sink_caps,
        "video/x-bayer,format=%s,width=%d,height=%d,framerate=30/1",
        element.sink_pad.formats[i], width, height);

    g_string_printf (sink_src,
        "filesrc location=/dev/zero blocksize=%d num-buffers=10 ! %s",
        blocksize, sink_caps->str);

    g_string_printf (pipeline, "%s ! tiovxfcvissmsc target=%s ! %s ! fakesink",
        sink_src->str, element.properties.target[0], src_caps->str);

    test_states_change_async (pipeline->str, TIOVXFC_STATE_CHANGE_ITERATIONS);
  }
}
GST_END_TEST

GST_START_TEST (test_resolutions_with_upscale_fail)
{
  TIOVXFCModeled element = { 0 };
  guint i = 0;

  g_autoptr (GString) pipeline = g_string_new ("");
  g_autoptr (GString) sink_caps = g_string_new ("");
  g_autoptr (GString) sink_src = g_string_new ("");
  g_autoptr (GString) src_caps = g_string_new ("");
  guint width = 0;
  guint height = 0;
  guint blocksize = 0;

  gst_tiovx_fc_modeling_init (&element);

  width =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.width_range->min,
      element.sink_pad.width_range->max);
  height =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.
      height_range->min, element.sink_pad.height_range->max);

  blocksize =
      gst_tiovx_fc_get_blocksize (width, height, element.sink_pad.formats[i],
      default_num_exposures);

  g_string_printf (sink_caps, "video/x-bayer,format=%s,width=%d,height=%d,framerate=30/1",
      element.sink_pad.formats[i], width, height);

  g_string_printf (sink_src, "filesrc location=/dev/zero blocksize=%d num-buffers=10 ! %s",
      blocksize, sink_caps->str);

  g_string_printf (src_caps, "video/x-raw,width=%d,height=%d", width + 1,
      height + 1);

  g_string_printf (pipeline,
      "%s ! tiovxfcvissmsc target=%s ! %s ! fakesink",
      sink_src->str, element.properties.target[0], src_caps->str);

  test_states_change_async_fail_success (pipeline->str, TIOVXFC_STATE_CHANGE_ITERATIONS);
}
GST_END_TEST

GST_START_TEST (test_resolutions_with_downscale_fail)
{
  TIOVXFCModeled element = { 0 };
  guint i = 0;

  g_autoptr (GString) pipeline = g_string_new ("");
  g_autoptr (GString) sink_caps = g_string_new ("");
  g_autoptr (GString) sink_src = g_string_new ("");
  g_autoptr (GString) src_caps = g_string_new ("");
  guint width = 0;
  guint height = 0;
  guint blocksize = 0;

  gst_tiovx_fc_modeling_init (&element);

  width =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.width_range->min,
      element.sink_pad.width_range->max);
  height =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.
      height_range->min, element.sink_pad.height_range->max);
  blocksize =
      gst_tiovx_fc_get_blocksize (width, height, element.sink_pad.formats[i],
      default_num_exposures);

  g_string_printf (sink_caps, "video/x-bayer,format=%s,width=%d,height=%d,framerate=30/1",
      element.sink_pad.formats[i], width, height);

  g_string_printf (sink_src, "filesrc location=/dev/zero blocksize=%d num-buffers=10 ! %s",
      blocksize, sink_caps->str);

  g_string_printf (src_caps, "video/x-raw,width=%d,height=%d, framerate=30/1 ", width - 1,
      height - 1);

  g_string_printf (pipeline,
      "%s ! tiovxfcvissmsc target=%s ! %s ! fakesink",
      sink_src->str, element.properties.target[0], src_caps->str);

  test_states_change_async_fail_success (pipeline->str, TIOVXFC_STATE_CHANGE_ITERATIONS);
}
GST_END_TEST

GST_START_TEST (test_target)
{
  TIOVXFCModeled element = { 0 };
  g_autoptr (GString) pipeline = g_string_new ("");
  g_autoptr (GString) sink_caps = g_string_new ("");
  g_autoptr (GString) sink_src = g_string_new ("");
  g_autoptr (GString) src_caps = g_string_new ("");
  guint width = 0;
  guint height = 0;
  guint blocksize = 0;
  guint format_idx = 0;
  guint j = 0;

  gst_tiovx_fc_modeling_init (&element);

  format_idx = g_random_int_range (0, TIOVXFC_INPUT_FORMATS_ARRAY_SIZE);

  width =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.
      width_range->min, element.sink_pad.width_range->max);
  height =
      gst_tiovx_fc_get_int_range_pair_value (element.sink_pad.
      height_range->min, element.sink_pad.height_range->max);
  blocksize =
      gst_tiovx_fc_get_blocksize (width, height, element.sink_pad.formats[format_idx],
      default_num_exposures);

  g_string_printf (sink_caps,
      "video/x-bayer,format=%s,width=%d,height=%d,framerate=30/1",
      element.sink_pad.formats[format_idx], width, height);
  g_string_printf (sink_src, "filesrc location=/dev/zero blocksize=%d num-buffers=10 ! %s",
      blocksize, sink_caps->str);

  g_string_printf (src_caps, "video/x-raw");

  for (j = 0; j < TIOVXFC_TARGET_ARRAY_SIZE; j++)
  {
    g_string_printf (pipeline, "%s ! tiovxfcvissmsc target=%s ! %s ! fakesink",
        sink_src->str, element.properties.target[j], src_caps->str);

    test_states_change_async (pipeline->str, TIOVXFC_STATE_CHANGE_ITERATIONS);
  }

}
GST_END_TEST

static Suite *
gst_tiovx_fc_suite (void)
{
  Suite *suite = suite_create ("tiovxfc");
  TCase *tc = tcase_create ("general");

  tcase_set_timeout (tc, 30);

  tcase_add_test (tc, test_target);
  tcase_add_test (tc, test_resolutions_with_downscale_fail);
  tcase_add_test (tc, test_resolutions_with_upscale_fail);
  tcase_add_test (tc, test_input_format_fail);
  tcase_add_test (tc, test_output_format_fail);
  tcase_add_test (tc, test_foreach_format);

  suite_add_tcase (suite, tc);

  return suite;
}

GST_CHECK_MAIN (gst_tiovx_fc);
