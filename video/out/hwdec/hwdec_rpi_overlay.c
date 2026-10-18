/*
 * This file is part of mpv.
 *
 * mpv is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * mpv is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with mpv.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Raspberry Pi video overlay: --hwdec=mmal frames (IMGFMT_MMAL, opaque
 * VideoCore buffers from FFmpeg's *_mmal decoders) go straight to the
 * VideoCore's video_render component, which shows them as a hardware
 * display layer (the HVS scales and converts them); the CPU and the GPU
 * never touch the pixels. OSD etc. is still rendered through the GPU API,
 * under the layer.
 *
 * This is mpv v0.32.0's video/out/opengl/hwdec_rpi.c (removed upstream with
 * vo_rpi), updated to the current hwdec/colorspace API. Changes:
 *   - opaque frames only: the old RAM-upload path took IMGFMT_420P, which
 *     would route every software-decoded frame to the overlay;
 *   - the layer's place on the screen comes from the --rpi-overlay-*
 *     options (settable at runtime: a libmpv client updates them when its
 *     window moves) instead of the old opengl-cb MPV_RPI_WINDOW resource;
 *   - the dst-rect test in overlay_frame() had lost its "!", so a moved
 *     video rectangle was never applied while the frame stayed the same.
 */

#include <stdbool.h>
#include <string.h>

#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/vc/mmal_vc_api.h>

#include <libavutil/rational.h>

#include "common/common.h"
#include "common/msg.h"
#include "options/m_config.h"
#include "options/m_option.h"
#include "video/mp_image.h"
#include "video/out/gpu/hwdec.h"

struct rpi_overlay_opts {
    int x, y;           // screen position of the render target's top-left pixel
    int layer;          // dispmanx layer (above the framebuffer's)
    int display;        // display number (0: the default display)
};

#define OPT_BASE_STRUCT struct rpi_overlay_opts
const struct m_sub_options rpi_overlay_conf = {
    .opts = (const struct m_option[]) {
        {"rpi-overlay-x", OPT_INT(x)},
        {"rpi-overlay-y", OPT_INT(y)},
        {"rpi-overlay-layer", OPT_INT(layer)},
        {"rpi-overlay-display", OPT_INT(display)},
        {0},
    },
    .size = sizeof(struct rpi_overlay_opts),
    .defaults = &(const struct rpi_overlay_opts) {
        .layer = 1,
    },
};
#undef OPT_BASE_STRUCT

struct priv {
    struct mp_log *log;
    struct m_config_cache *opts_cache;

    struct mp_image_params params;

    MMAL_COMPONENT_T *renderer;
    bool renderer_enabled;

    struct mp_image *current_frame;

    struct mp_rect src, dst;
    struct rpi_overlay_opts cur_window; // what the layer was last placed with
};

// Magic alignments (in pixels) expected by the MMAL internals.
#define ALIGN_W 32
#define ALIGN_H 16

static MMAL_FOURCC_T map_csp(enum pl_color_system sys)
{
    switch (sys) {
    case PL_COLOR_SYSTEM_BT_601:     return MMAL_COLOR_SPACE_ITUR_BT601;
    case PL_COLOR_SYSTEM_BT_709:     return MMAL_COLOR_SPACE_ITUR_BT709;
    case PL_COLOR_SYSTEM_SMPTE_240M: return MMAL_COLOR_SPACE_SMPTE240M;
    default:                         return MMAL_COLOR_SPACE_UNKNOWN;
    }
}

static void control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    mmal_buffer_header_release(buffer);
}

static void input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    struct mp_image *mpi = buffer->user_data;
    talloc_free(mpi);
}

static void disable_renderer(struct ra_hwdec *hw)
{
    struct priv *p = hw->priv;

    if (p->renderer_enabled) {
        mmal_port_disable(p->renderer->control);
        mmal_port_disable(p->renderer->input[0]);

        mmal_port_flush(p->renderer->control);
        mmal_port_flush(p->renderer->input[0]);

        mmal_component_disable(p->renderer);
    }
    p->renderer_enabled = false;
}

// check_window_only: assume params and dst/src rc are unchanged
static void update_overlay(struct ra_hwdec *hw, bool check_window_only)
{
    struct priv *p = hw->priv;
    MMAL_PORT_T *input = p->renderer->input[0];
    struct mp_rect src = p->src;
    struct mp_rect dst = p->dst;

    m_config_cache_update(p->opts_cache);
    struct rpi_overlay_opts *z = p->opts_cache->opts;

    if (check_window_only && memcmp(z, &p->cur_window, sizeof(p->cur_window)) == 0)
        return;

    p->cur_window = *z;

    int rotate[] = {MMAL_DISPLAY_ROT0,
                    MMAL_DISPLAY_ROT90,
                    MMAL_DISPLAY_ROT180,
                    MMAL_DISPLAY_ROT270};

    int src_w = src.x1 - src.x0, src_h = src.y1 - src.y0,
        dst_w = dst.x1 - dst.x0, dst_h = dst.y1 - dst.y0;
    if (src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0)
        return;
    int p_x, p_y;
    av_reduce(&p_x, &p_y, dst_w * src_h, src_w * dst_h, 16000);
    MMAL_DISPLAYREGION_T dr = {
        .hdr = { .id = MMAL_PARAMETER_DISPLAYREGION,
                 .size = sizeof(MMAL_DISPLAYREGION_T), },
        .src_rect = { .x = src.x0, .y = src.y0,
                      .width = src_w, .height = src_h },
        .dest_rect = { .x = dst.x0 + z->x, .y = dst.y0 + z->y,
                       .width = dst_w, .height = dst_h },
        .layer = z->layer,
        .display_num = z->display,
        .pixel_x = p_x,
        .pixel_y = p_y,
        .transform = rotate[(p->params.rotate % 360) / 90],
        .fullscreen = 0,
        .set = MMAL_DISPLAY_SET_SRC_RECT | MMAL_DISPLAY_SET_DEST_RECT |
               MMAL_DISPLAY_SET_LAYER | MMAL_DISPLAY_SET_NUM |
               MMAL_DISPLAY_SET_PIXEL | MMAL_DISPLAY_SET_TRANSFORM |
               MMAL_DISPLAY_SET_FULLSCREEN,
    };

    if (p->params.rotate % 180 == 90) {
        MPSWAP(int, dr.src_rect.x, dr.src_rect.y);
        MPSWAP(int, dr.src_rect.width, dr.src_rect.height);
    }

    if (mmal_port_parameter_set(input, &dr.hdr))
        MP_WARN(p, "could not set video rectangle\n");
}

static int enable_renderer(struct ra_hwdec *hw)
{
    struct priv *p = hw->priv;
    MMAL_PORT_T *input = p->renderer->input[0];
    struct mp_image_params *params = &p->params;

    if (p->renderer_enabled)
        return 0;

    if (params->imgfmt != IMGFMT_MMAL)
        return -1;

    input->format->encoding = MMAL_ENCODING_OPAQUE;
    input->format->es->video.width = MP_ALIGN_UP(params->w, ALIGN_W);
    input->format->es->video.height = MP_ALIGN_UP(params->h, ALIGN_H);
    input->format->es->video.crop = (MMAL_RECT_T){0, 0, params->w, params->h};
    input->format->es->video.par = (MMAL_RATIONAL_T){params->p_w, params->p_h};
    input->format->es->video.color_space = map_csp(params->repr.sys);

    if (mmal_port_format_commit(input))
        return -1;

    input->buffer_num = MPMAX(input->buffer_num_min,
                              input->buffer_num_recommended) + 3;
    input->buffer_size = MPMAX(input->buffer_size_min,
                               input->buffer_size_recommended);

    update_overlay(hw, false);

    p->renderer_enabled = true;

    if (mmal_port_enable(p->renderer->control, control_port_cb))
        return -1;

    if (mmal_port_enable(input, input_port_cb))
        return -1;

    if (mmal_component_enable(p->renderer)) {
        MP_FATAL(hw, "Failed to enable video renderer.\n");
        return -1;
    }

    return 0;
}

static int overlay_frame(struct ra_hwdec *hw, struct mp_image *hw_image,
                         struct mp_rect *src, struct mp_rect *dst, bool newframe)
{
    struct priv *p = hw->priv;

    if (hw_image && !mp_image_params_equal(&p->params, &hw_image->params)) {
        p->params = hw_image->params;

        disable_renderer(hw);
        mp_image_unrefp(&p->current_frame);

        if (enable_renderer(hw) < 0)
            return -1;
    }

    if (hw_image && p->current_frame && !newframe) {
        if (!mp_rect_equals(&p->src, src) || !mp_rect_equals(&p->dst, dst)) {
            p->src = *src;
            p->dst = *dst;
            update_overlay(hw, false);
        } else {
            update_overlay(hw, true);   // the window may have moved
        }
        return 0; // don't reupload
    }

    mp_image_unrefp(&p->current_frame);

    if (!hw_image) {
        disable_renderer(hw);
        return 0;
    }

    if (enable_renderer(hw) < 0)
        return -1;

    if (!mp_rect_equals(&p->src, src) || !mp_rect_equals(&p->dst, dst)) {
        p->src = *src;
        p->dst = *dst;
        update_overlay(hw, false);
    } else {
        update_overlay(hw, true);
    }

    struct mp_image *mpi = mp_image_new_ref(hw_image);
    if (!mpi) {
        disable_renderer(hw);
        return -1;
    }

    // Keep a reference as the current frame (the renderer holds the buffer
    // being displayed; this one keeps the mp_image alive for redraws).
    p->current_frame = mp_image_new_ref(hw_image);

    MMAL_BUFFER_HEADER_T *ref = (void *)mpi->planes[3];

    // Assume this field is free for use by us.
    ref->user_data = mpi;

    if (mmal_port_send_buffer(p->renderer->input[0], ref)) {
        MP_ERR(hw, "could not queue picture!\n");
        talloc_free(mpi);
        return -1;
    }

    return 0;
}

static void destroy(struct ra_hwdec *hw)
{
    struct priv *p = hw->priv;

    disable_renderer(hw);
    mp_image_unrefp(&p->current_frame);

    if (p->renderer)
        mmal_component_release(p->renderer);

    mmal_vc_deinit();
}

static int create(struct ra_hwdec *hw)
{
    struct priv *p = hw->priv;
    p->log = hw->log;
    p->opts_cache = m_config_cache_alloc(p, hw->global, &rpi_overlay_conf);

    bcm_host_init();

    if (mmal_vc_init()) {
        MP_FATAL(hw, "Could not initialize MMAL.\n");
        return -1;
    }

    if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &p->renderer))
    {
        MP_FATAL(hw, "Could not create MMAL renderer.\n");
        mmal_vc_deinit();
        return -1;
    }

    return 0;
}

const struct ra_hwdec_driver ra_hwdec_rpi_overlay = {
    .name = "rpi-overlay",
    .priv_size = sizeof(struct priv),
    .imgfmts = {IMGFMT_MMAL, 0},
    .init = create,
    .overlay_frame = overlay_frame,
    .uninit = destroy,
};
