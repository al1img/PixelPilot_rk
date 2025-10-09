
#include <stddef.h>
#include <sys/time.h>
#include <cassert>

#include <im2d.h>
#include <RgaUtils.h>
#include <spdlog/spdlog.h>

#include "encode.h"

/***********************************************************************************************************************
 * Vars
 **********************************************************************************************************************/

encode_ctx encode;

/***********************************************************************************************************************
 * Function declarations
 **********************************************************************************************************************/

void *__ENCODE_THREAD__(void *param);
MPP_RET set_mpp_encoding_parameters();
int blend_osd();

/***********************************************************************************************************************
 * Static functions
 **********************************************************************************************************************/

int64_t get_cur_us() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec * 1000000 + tv.tv_usec;
}


/***********************************************************************************************************************
 * Public functions
 **********************************************************************************************************************/

void init_encode(Dvr *dvr, MppCodingType mpp_type, int frame_rate) {
	encode.mpp_type = mpp_type;
	encode.frame_rate = frame_rate;
	encode.dvr = dvr;
	// Initialize encoder
	int ret = mpp_check_support_format(MPP_CTX_ENC, encode.mpp_type);
	assert(!ret);
	ret = mpp_create(&encode.ctx, &encode.mpi);
	assert(!ret);
	ret = mpp_init(encode.ctx, MPP_CTX_ENC, mpp_type);
	assert(!ret);
	MppPollType timeout = MPP_POLL_BLOCK;
	ret = encode.mpi->control(encode.ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
	assert(!ret);
	// Create encode thread
	ret = pthread_mutex_init(&encode.mutex, NULL);
	assert(!ret);
	ret = pthread_cond_init(&encode.cond, NULL);
	assert(!ret);
	ret = pthread_create(&encode.tid, NULL, __ENCODE_THREAD__, NULL);
	assert(!ret);
}

void release_encode() {
	spdlog::info("Release encode");

	int ret = pthread_mutex_lock(&encode.mutex);
	assert(!ret);
	encode.close = true;
	ret = pthread_cond_signal(&encode.cond);
	assert(!ret);
	ret = pthread_mutex_unlock(&encode.mutex);
	assert(!ret);
	ret = pthread_join(encode.tid, NULL);
	assert(!ret);
	ret = pthread_cond_destroy(&encode.cond);
	assert(!ret);
	ret = pthread_mutex_destroy(&encode.mutex);
	assert(!ret);

	release_encode_buffers();

	ret = encode.mpi->reset(encode.ctx);
	assert(!ret);

	mpp_destroy(encode.ctx);

	spdlog::info("Encode released");
}

void release_encode_buffers() {
	if (!encode.frm_grp) {
		return;
	}

	spdlog::debug("Freeing current encode mpp_buffer_group");

	for (int i = 0; i < MAX_ENCODE_BUFS; i++) {
		mpp_buffer_put(encode.frm_buf[i]);
	}
	mpp_buffer_put(encode.osd_buf);
	mpp_buffer_group_put(encode.frm_grp);
	encode.frm_grp = NULL;

	encode.cur_buf = NULL;
	encode.prev_buf = NULL;
}

void create_encode_buffers(uint32_t width, uint32_t height, uint32_t hstride, uint32_t vstride, MppFrameFormat fmt) {
	int ret = mpp_buffer_group_get_internal(&encode.frm_grp, MPP_BUFFER_TYPE_DRM);
	assert(!ret);
	for (int i = 0; i < MAX_ENCODE_BUFS; i++) {
		ret = mpp_buffer_get(encode.frm_grp, &encode.frm_buf[i], CODEC_ALIGN(hstride, 64) * CODEC_ALIGN(vstride, 64) * 2);
		assert(!ret);
	}
	ret = mpp_buffer_get(encode.frm_grp, &encode.osd_buf, CODEC_ALIGN(hstride, 64) * CODEC_ALIGN(vstride, 64) * 4);
	assert(!ret);

	encode.frm_fmt = fmt;
	encode.frm_width = width;
	encode.frm_height = height;
	encode.hor_stride = hstride;
	encode.ver_stride = vstride;

	ret = set_mpp_encoding_parameters();
	assert(!ret);
}

void update_encode_buffer(MppBuffer buffer) {
	int ret = pthread_mutex_lock(&encode.mutex);
	assert(!ret);

	encode.prev_buf = encode.cur_buf;
	encode.cur_buf = buffer;

	ret = pthread_mutex_unlock(&encode.mutex);
	assert(!ret);
}

void process_encode() {
	int ret = pthread_mutex_lock(&encode.mutex);
	assert(!ret);
	ret = blend_osd();
	assert(!ret);
	encode.ready = true;
	ret = pthread_cond_signal(&encode.cond);
	assert(!ret);
	ret = pthread_mutex_unlock(&encode.mutex);
	assert(!ret);
}

int copy_osd_buf(const modeset_buf& osd_buf) {
	MppBufferInfo dst_info;
	rga_buffer_t src_img, dst_img;
	rga_buffer_handle_t src_handle, dst_handle;
	int64_t start, end;
	uint32_t fmt = RK_FORMAT_RGBA_8888;

	start = get_cur_us();

	int ret = mpp_buffer_info_get(encode.osd_buf, &dst_info);
	if (ret) {
		spdlog::error("get dst buf info failed");
		return ret;
	}

	src_handle = importbuffer_virtualaddr(osd_buf.map, osd_buf.size);
	dst_handle = importbuffer_fd(dst_info.fd, dst_info.size);

	if (!src_handle || !dst_handle) {
		spdlog::error("import buffer failed");
		ret = -1;
		goto release_buffers;
	}

	src_img = wrapbuffer_handle(src_handle, osd_buf.width, osd_buf.height, fmt);
	dst_img = wrapbuffer_handle(dst_handle, encode.frm_width, encode.frm_height, fmt, encode.hor_stride, encode.ver_stride);

	ret = imcheck(src_img, dst_img, {}, {});
	if (IM_STATUS_NOERROR != ret) {
		spdlog::error("resize check failed");
		ret = -1;
		goto release_buffers;
	}

	ret = imresize(src_img, dst_img);
	if (ret != IM_STATUS_SUCCESS) {
		spdlog::error("resize failed: {}", ret);
		ret = -1;
	} else {
		ret = 0;
	}
/*
	im_opt opt {};

	opt.version = RGA_CURRENT_API_VERSION;
	opt.interp = IM_INTERP(IM_INTERP_CUBIC, IM_INTERP_CUBIC);

	ret = improcess(src_img, dst_img, {}, {}, {}, {}, 0, NULL, &opt, 0);
	if (ret != IM_STATUS_SUCCESS) {
		spdlog::error("resize failed: {}", ret);
		ret = -1;
	} else {
		ret = 0;
	}
*/
release_buffers:
	if (src_handle) {
		releasebuffer_handle(src_handle);
	}

	if (dst_handle) {
		releasebuffer_handle(dst_handle);
	}

	end = get_cur_us();

	spdlog::debug("RGA copy time: {} us", end - start);

	return ret;
}

/***********************************************************************************************************************
 * Private functions
 **********************************************************************************************************************/

int blend_osd() {
	int ret = 0;
	int fg_width, fg_height, fg_format;
	int bg_width, bg_height, bg_format;
	int output_width, output_height, output_format;
	void *fg_buf, *bg_buf, *output_buf;
	int fg_buf_size, bg_buf_size, output_buf_size;
	rga_buffer_t fg_img, bg_img, output_img;
	im_rect fg_rect, bg_rect, output_rect;
	rga_buffer_handle_t fg_handle, bg_handle, output_handle;
	MppBufferInfo fg_info, bg_info, output_info;
	int64_t start, end;
	int usage = 0;

	start = get_cur_us();

	ret = mpp_buffer_info_get(encode.prev_buf, &fg_info);
	if (ret) {
		spdlog::error("get fg buf info failed");
		return ret;
	}

	ret = mpp_buffer_info_get(encode.osd_buf, &bg_info);
	if (ret) {
		spdlog::error("get bg buf info failed");
		return ret;
	}

	ret = mpp_buffer_info_get(encode.frm_buf[encode.frm_idx], &output_info);
	if (ret) {
		spdlog::error("get output buf info failed");
		return ret;
	}


	fg_width = encode.frm_width;
	fg_height = encode.frm_height;
	fg_format = encode.frm_fmt == MPP_FMT_YUV420SP ? RK_FORMAT_YCrCb_420_SP : RK_FORMAT_YCrCb_420_SP_10B;

	bg_width = encode.frm_width;
	bg_height = encode.frm_height;
	bg_format = RK_FORMAT_RGBA_8888;

	output_width = fg_width;
	output_height = fg_height;
	output_format = fg_format;

	fg_buf_size =fg_info.size;
	bg_buf_size = bg_info.size;
	output_buf_size = output_info.size;

	fg_buf = fg_info.ptr;
	bg_buf = bg_info.ptr;
	output_buf = output_info.ptr;

	fg_handle = importbuffer_fd(fg_info.fd, fg_buf_size);
	bg_handle = importbuffer_fd(bg_info.fd, bg_buf_size);
	output_handle = importbuffer_fd(output_info.fd, output_buf_size);
	if (fg_handle == 0 || bg_handle == 0 || output_handle == 0) {
		ret = -1;
		spdlog::error("import buffer failed");
		goto release_buffers;
	}

	fg_img = wrapbuffer_handle(fg_handle, fg_width, fg_height, fg_format, encode.hor_stride, encode.ver_stride);
	bg_img = wrapbuffer_handle(bg_handle, bg_width, bg_height, bg_format);
	output_img = wrapbuffer_handle(output_handle, output_width, output_height, output_format, encode.hor_stride, encode.ver_stride);

	/*
	* Configure the blended rectangular area here.
	*      Here is intercepted from the foreground image (100, 200) as the starting point,
	*  and a rectangle with the same resolution as the background image is blended with
	*  the background image, and finally output to the output layer where (100, 200) is
	*  the starting point.
	*      fg_img => src_channel
	*      bg_img => src1_channel
	*      output_img => dst_channel
		---------------------------        --------------      ---------------------------
		|         fg_img          |        |   bg_img/  |      |       output_img        |
		|     --------------      |        |   bg_rect  |      |     --------------      |
		|     |            |      |        |            |      |     |            |      |
		|     |   fg_rect  |      |    +   --------------  =>  |     | output_rect|      |
		|     |            |      |                            |     |(bg over fg)|      |
		|     --------------      |                            |     --------------      |
		|                         |                            |                         |
		---------------------------                            ---------------------------
	*/

	fg_rect.x = 0;
	fg_rect.y = 0;
	fg_rect.width = fg_width;
	fg_rect.height = fg_height;

	bg_rect.x = 0;
	bg_rect.y = 0;
	bg_rect.width = bg_width;
	bg_rect.height = bg_height;

	output_rect.x = 0;
	output_rect.y = 0;
	output_rect.width = output_width;
	output_rect.height = output_height;

	ret = imcheck_composite(fg_img, output_img, bg_img, fg_rect, output_rect, bg_rect);
	if (IM_STATUS_NOERROR != ret) {
		ret = -1;
		spdlog::error("check failed");
		goto release_buffers;
	}

	usage = IM_SYNC | IM_ALPHA_BLEND_DST_OVER | IM_ALPHA_BLEND_PRE_MUL;

	ret = improcess(fg_img, output_img, bg_img, fg_rect, output_rect, bg_rect, -1, NULL, NULL, usage);
	if (ret != IM_STATUS_SUCCESS) {
		spdlog::error("blend failed");
		ret = -1;
	} else {
		ret = 0;
	}

release_buffers:
	if (fg_handle)
		releasebuffer_handle(fg_handle);
	if (bg_handle)
		releasebuffer_handle(bg_handle);
	if (output_handle)
		releasebuffer_handle(output_handle);

	end = get_cur_us();

	spdlog::debug("RGA blend time: {} us", end - start);

	return ret;
}

#if 0 // keep for debug purposes

int convert_output() {
	MppBufferInfo src_info, dst_info;
	rga_buffer_t src_img, dst_img;
	rga_buffer_handle_t src_handle, dst_handle;
	int src_fmt, dst_fmt;
	int64_t start, end;

	start = get_cur_us();

	MppBuffer src_buf = encode.frm_buf[encode.frm_idx];
	MppBuffer dst_buf = encode.frm_buf[(encode.frm_idx + 1) % MAX_ENCODE_BUFS];

	int ret = mpp_buffer_info_get(src_buf, &src_info);
	if (ret) {
		spdlog::error("get src buf info failed");
		return ret;
	}

	ret = mpp_buffer_info_get(dst_buf, &dst_info);
	if (ret) {
		spdlog::error("get dst buf info failed");
		return ret;
	}

	src_handle = importbuffer_fd(src_info.fd, src_info.size);
	dst_handle = importbuffer_fd(dst_info.fd, dst_info.size);

	if (!src_handle || !dst_handle) {
		spdlog::error("import buffer failed");
		ret = -1;
		goto release_buffers;
	}

	src_fmt = encode.frm_fmt == MPP_FMT_YUV420SP ? RK_FORMAT_YCrCb_420_SP : RK_FORMAT_YCrCb_420_SP_10B;
	dst_fmt = RK_FORMAT_RGBA_8888;

	src_img = wrapbuffer_handle(src_handle, encode.frm_width, encode.frm_height, src_fmt);
	dst_img = wrapbuffer_handle(dst_handle, encode.frm_width, encode.frm_height, dst_fmt);

	ret = imcheck(src_img, dst_img, {}, {});
	if (IM_STATUS_NOERROR != ret) {
		spdlog::error("convert check failed");
		ret = -1;
		goto release_buffers;
	}

	ret = imcvtcolor(src_img, dst_img, src_fmt, dst_fmt);
	if (ret != IM_STATUS_SUCCESS) {
		spdlog::error("convert failed: {}", ret);
		ret = -1;
	} else {
		ret = 0;
	}

release_buffers:
	if (src_handle) {
		releasebuffer_handle(src_handle);
	}

	if (dst_handle) {
		releasebuffer_handle(dst_handle);
	}

	end = get_cur_us();

	spdlog::debug("RGA convert time: {} us", end - start);

	return ret;
}

#endif

MPP_RET mpi_enc_gen_ref_cfg(MppEncRefCfg ref, RK_S32 gop_mode)
{
    MppEncRefLtFrmCfg lt_ref[4];
    MppEncRefStFrmCfg st_ref[16];
    RK_S32 lt_cnt = 0;
    RK_S32 st_cnt = 0;
    MPP_RET ret = MPP_OK;

    memset(&lt_ref, 0, sizeof(lt_ref));
    memset(&st_ref, 0, sizeof(st_ref));

    switch (gop_mode) {
    case 3 : {
        // tsvc4
        //      /-> P1      /-> P3        /-> P5      /-> P7
        //     /           /             /           /
        //    //--------> P2            //--------> P6
        //   //                        //
        //  ///---------------------> P4
        // ///
        // P0 ------------------------------------------------> P8
        lt_cnt = 1;

        /* set 8 frame lt-ref gap */
        lt_ref[0].lt_idx        = 0;
        lt_ref[0].temporal_id   = 0;
        lt_ref[0].ref_mode      = REF_TO_PREV_LT_REF;
        lt_ref[0].lt_gap        = 8;
        lt_ref[0].lt_delay      = 0;

        st_cnt = 9;
        /* set tsvc4 st-ref struct */
        /* st 0 layer 0 - ref */
        st_ref[0].is_non_ref    = 0;
        st_ref[0].temporal_id   = 0;
        st_ref[0].ref_mode      = REF_TO_TEMPORAL_LAYER;
        st_ref[0].ref_arg       = 0;
        st_ref[0].repeat        = 0;
        /* st 1 layer 3 - non-ref */
        st_ref[1].is_non_ref    = 1;
        st_ref[1].temporal_id   = 3;
        st_ref[1].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[1].ref_arg       = 0;
        st_ref[1].repeat        = 0;
        /* st 2 layer 2 - ref */
        st_ref[2].is_non_ref    = 0;
        st_ref[2].temporal_id   = 2;
        st_ref[2].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[2].ref_arg       = 0;
        st_ref[2].repeat        = 0;
        /* st 3 layer 3 - non-ref */
        st_ref[3].is_non_ref    = 1;
        st_ref[3].temporal_id   = 3;
        st_ref[3].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[3].ref_arg       = 0;
        st_ref[3].repeat        = 0;
        /* st 4 layer 1 - ref */
        st_ref[4].is_non_ref    = 0;
        st_ref[4].temporal_id   = 1;
        st_ref[4].ref_mode      = REF_TO_PREV_LT_REF;
        st_ref[4].ref_arg       = 0;
        st_ref[4].repeat        = 0;
        /* st 5 layer 3 - non-ref */
        st_ref[5].is_non_ref    = 1;
        st_ref[5].temporal_id   = 3;
        st_ref[5].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[5].ref_arg       = 0;
        st_ref[5].repeat        = 0;
        /* st 6 layer 2 - ref */
        st_ref[6].is_non_ref    = 0;
        st_ref[6].temporal_id   = 2;
        st_ref[6].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[6].ref_arg       = 0;
        st_ref[6].repeat        = 0;
        /* st 7 layer 3 - non-ref */
        st_ref[7].is_non_ref    = 1;
        st_ref[7].temporal_id   = 3;
        st_ref[7].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[7].ref_arg       = 0;
        st_ref[7].repeat        = 0;
        /* st 8 layer 0 - ref */
        st_ref[8].is_non_ref    = 0;
        st_ref[8].temporal_id   = 0;
        st_ref[8].ref_mode      = REF_TO_TEMPORAL_LAYER;
        st_ref[8].ref_arg       = 0;
        st_ref[8].repeat        = 0;
    } break;
    case 2 : {
        // tsvc3
        //     /-> P1      /-> P3
        //    /           /
        //   //--------> P2
        //  //
        // P0/---------------------> P4
        lt_cnt = 0;

        st_cnt = 5;
        /* set tsvc4 st-ref struct */
        /* st 0 layer 0 - ref */
        st_ref[0].is_non_ref    = 0;
        st_ref[0].temporal_id   = 0;
        st_ref[0].ref_mode      = REF_TO_TEMPORAL_LAYER;
        st_ref[0].ref_arg       = 0;
        st_ref[0].repeat        = 0;
        /* st 1 layer 2 - non-ref */
        st_ref[1].is_non_ref    = 1;
        st_ref[1].temporal_id   = 2;
        st_ref[1].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[1].ref_arg       = 0;
        st_ref[1].repeat        = 0;
        /* st 2 layer 1 - ref */
        st_ref[2].is_non_ref    = 0;
        st_ref[2].temporal_id   = 1;
        st_ref[2].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[2].ref_arg       = 0;
        st_ref[2].repeat        = 0;
        /* st 3 layer 2 - non-ref */
        st_ref[3].is_non_ref    = 1;
        st_ref[3].temporal_id   = 2;
        st_ref[3].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[3].ref_arg       = 0;
        st_ref[3].repeat        = 0;
        /* st 4 layer 0 - ref */
        st_ref[4].is_non_ref    = 0;
        st_ref[4].temporal_id   = 0;
        st_ref[4].ref_mode      = REF_TO_TEMPORAL_LAYER;
        st_ref[4].ref_arg       = 0;
        st_ref[4].repeat        = 0;
    } break;
    case 1 : {
        // tsvc2
        //   /-> P1
        //  /
        // P0--------> P2
        lt_cnt = 0;

        st_cnt = 3;
        /* set tsvc4 st-ref struct */
        /* st 0 layer 0 - ref */
        st_ref[0].is_non_ref    = 0;
        st_ref[0].temporal_id   = 0;
        st_ref[0].ref_mode      = REF_TO_TEMPORAL_LAYER;
        st_ref[0].ref_arg       = 0;
        st_ref[0].repeat        = 0;
        /* st 1 layer 2 - non-ref */
        st_ref[1].is_non_ref    = 1;
        st_ref[1].temporal_id   = 1;
        st_ref[1].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[1].ref_arg       = 0;
        st_ref[1].repeat        = 0;
        /* st 2 layer 1 - ref */
        st_ref[2].is_non_ref    = 0;
        st_ref[2].temporal_id   = 0;
        st_ref[2].ref_mode      = REF_TO_PREV_REF_FRM;
        st_ref[2].ref_arg       = 0;
        st_ref[2].repeat        = 0;
    } break;
    default : {
		spdlog::error("unsupported gop mode {}", gop_mode);
    } break;
    }

    if (lt_cnt || st_cnt) {
        ret = mpp_enc_ref_cfg_set_cfg_cnt(ref, lt_cnt, st_cnt);

        if (lt_cnt)
            ret = mpp_enc_ref_cfg_add_lt_cfg(ref, lt_cnt, lt_ref);

        if (st_cnt)
            ret = mpp_enc_ref_cfg_add_st_cfg(ref, st_cnt, st_ref);

        /* check and get dpb size */
        ret = mpp_enc_ref_cfg_check(ref);
    }

    return ret;
}

MPP_RET mpi_enc_gen_smart_gop_ref_cfg(MppEncRefCfg ref, RK_S32 gop_len, RK_S32 vi_len)
{
    MppEncRefLtFrmCfg lt_ref[4];
    MppEncRefStFrmCfg st_ref[16];
    RK_S32 lt_cnt = 1;
    RK_S32 st_cnt = 8;
    RK_S32 pos = 0;
    MPP_RET ret = MPP_OK;

    memset(&lt_ref, 0, sizeof(lt_ref));
    memset(&st_ref, 0, sizeof(st_ref));

    ret = mpp_enc_ref_cfg_set_cfg_cnt(ref, lt_cnt, st_cnt);

    /* set 8 frame lt-ref gap */
    lt_ref[0].lt_idx        = 0;
    lt_ref[0].temporal_id   = 0;
    lt_ref[0].ref_mode      = REF_TO_PREV_LT_REF;
    lt_ref[0].lt_gap        = gop_len;
    lt_ref[0].lt_delay      = 0;

    ret = mpp_enc_ref_cfg_add_lt_cfg(ref, 1, lt_ref);

    /* st 0 layer 0 - ref */
    st_ref[pos].is_non_ref  = 0;
    st_ref[pos].temporal_id = 0;
    st_ref[pos].ref_mode    = REF_TO_PREV_INTRA;
    st_ref[pos].ref_arg     = 0;
    st_ref[pos].repeat      = 0;
    pos++;

    /* st 1 layer 1 - non-ref */
    if (vi_len > 1) {
        st_ref[pos].is_non_ref  = 0;
        st_ref[pos].temporal_id = 0;
        st_ref[pos].ref_mode    = REF_TO_PREV_REF_FRM;
        st_ref[pos].ref_arg     = 0;
        st_ref[pos].repeat      = vi_len - 2;
        pos++;
    }

    st_ref[pos].is_non_ref  = 0;
    st_ref[pos].temporal_id = 0;
    st_ref[pos].ref_mode    = REF_TO_PREV_INTRA;
    st_ref[pos].ref_arg     = 0;
    st_ref[pos].repeat      = 0;
    pos++;

    ret = mpp_enc_ref_cfg_add_st_cfg(ref, pos, st_ref);

    /* check and get dpb size */
    ret = mpp_enc_ref_cfg_check(ref);

    return ret;
}

MPP_RET set_mpp_encoding_parameters() {
	// Config
	RK_S32 rc_mode = MPP_ENC_RC_MODE_VBR;

	RK_S32 fps_in_flex = 0;
	RK_S32 fps_in_den = 0;
	RK_S32 fps_in_num = encode.frame_rate;
	RK_S32 fps_out_flex = 0;
	RK_S32 fps_out_den = 0;
	RK_S32 fps_out_num = encode.frame_rate;

	RK_S32 bps_min = 0;
	RK_S32 bps_max = 0;
	RK_S32 bps = 0;

    RK_S32 qp_init = 0;
    RK_S32 qp_min = 0;
    RK_S32 qp_max = 0;
    RK_S32 qp_min_i = 0;
    RK_S32 qp_max_i = 0;

    RK_S32 fqp_min_i = 0;
    RK_S32 fqp_min_p = 0;
    RK_S32 fqp_max_i = 0;
    RK_S32 fqp_max_p = 0;

	RK_U32 rotation = 0;
	RK_U32 mirroring = 0;
	RK_U32 flip = 0;

	RK_S32 gop_mode = 0;
	RK_S32 gop_len = 0;
	RK_S32 vi_len = 0;

	RK_S32 cu_qp_delta_depth = 0;

	MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
	MppEncSeiMode sei_mode = MPP_ENC_SEI_MODE_DISABLE;

	// Vars
	MppEncCfg cfg = NULL;
	MppEncRefCfg ref = NULL;

	mpp_enc_cfg_init(&cfg);
	// get default config from encoder context
	MPP_RET ret = encode.mpi->control(encode.ctx, MPP_ENC_GET_CFG, cfg);
	if (ret) {
		spdlog::warn("{} failed to get encoder cfg ret {}", encode.ctx, ret);
		return ret;
	}

    /* setup default parameter */
    if (fps_in_den == 0)
        fps_in_den = 1;
    if (fps_in_num == 0)
        fps_in_num = 30;
    if (fps_out_den == 0)
        fps_out_den = 1;
    if (fps_out_num == 0)
        fps_out_num = 30;

	if (!bps) {
		bps = encode.frm_width * encode.frm_height / 8 * (fps_out_num / fps_out_den);
	}

	spdlog::debug("Setup encoding parameters: {}x{} {} fps, target bps {}", encode.frm_width, encode.frm_height, fps_out_num / fps_out_den, bps);

	/* setup preprocess parameters */
	mpp_enc_cfg_set_s32(cfg, "prep:width", encode.frm_width);
	mpp_enc_cfg_set_s32(cfg, "prep:height", encode.frm_height);
	mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", encode.hor_stride);
	mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", encode.ver_stride);
	mpp_enc_cfg_set_s32(cfg, "prep:format", encode.frm_fmt);
	mpp_enc_cfg_set_s32(cfg, "prep:range", MPP_FRAME_RANGE_JPEG);

	mpp_enc_cfg_set_s32(cfg, "prep:mirroring", mirroring);
	mpp_enc_cfg_set_s32(cfg, "prep:rotation", rotation);
	mpp_enc_cfg_set_s32(cfg, "prep:flip", flip);

	/* setup rate control parameters */
	mpp_enc_cfg_set_s32(cfg, "rc:mode", rc_mode);
	mpp_enc_cfg_set_u32(cfg, "rc:max_reenc_times", 0);
	mpp_enc_cfg_set_u32(cfg, "rc:super_mode", 0);

	/* fix input / output frame rate */
	mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", fps_in_flex);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", fps_in_num);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denom", fps_in_den);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", fps_out_flex);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", fps_out_num);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denom", fps_out_den);

	/* drop frame or not when bitrate overflow */
	mpp_enc_cfg_set_u32(cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
	mpp_enc_cfg_set_u32(cfg, "rc:drop_thd", 20);        /* 20% of max bps */
	mpp_enc_cfg_set_u32(cfg, "rc:drop_gap", 1);         /* Do not continuous drop frame */

	/* setup bitrate for different rc_mode */
	mpp_enc_cfg_set_s32(cfg, "rc:bps_target", bps);
	switch (rc_mode) {
	case MPP_ENC_RC_MODE_FIXQP : {
		/* do not setup bitrate on FIXQP mode */
	} break;
	case MPP_ENC_RC_MODE_CBR : {
		/* CBR mode has narrow bound */
		mpp_enc_cfg_set_s32(cfg, "rc:bps_max", bps_max ? bps_max : bps * 17 / 16);
		mpp_enc_cfg_set_s32(cfg, "rc:bps_min", bps_min ? bps_min : bps * 15 / 16);
	} break;
	case MPP_ENC_RC_MODE_VBR :
	case MPP_ENC_RC_MODE_AVBR : {
		/* VBR mode has wide bound */
		mpp_enc_cfg_set_s32(cfg, "rc:bps_max", bps_max ? bps_max : bps * 17 / 16);
		mpp_enc_cfg_set_s32(cfg, "rc:bps_min", bps_min ? bps_min : bps * 1 / 16);
	} break;
	default : {
		/* default use CBR mode */
		mpp_enc_cfg_set_s32(cfg, "rc:bps_max", bps_max ? bps_max : bps * 17 / 16);
		mpp_enc_cfg_set_s32(cfg, "rc:bps_min", bps_min ? bps_min : bps * 15 / 16);
	} break;
	}

	/* setup qp for different codec and rc_mode */
	switch (encode.mpp_type) {
	case MPP_VIDEO_CodingAVC :
	case MPP_VIDEO_CodingHEVC : {
		switch (rc_mode) {
		case MPP_ENC_RC_MODE_FIXQP : {
			RK_S32 fix_qp = qp_init;

			mpp_enc_cfg_set_s32(cfg, "rc:qp_init", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_max", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_min", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 0);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_min_i", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_max_i", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_min_p", fix_qp);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_max_p", fix_qp);
		} break;
		case MPP_ENC_RC_MODE_CBR :
		case MPP_ENC_RC_MODE_VBR :
		case MPP_ENC_RC_MODE_AVBR :
		case MPP_ENC_RC_MODE_SMTRC : {
			mpp_enc_cfg_set_s32(cfg, "rc:qp_init", qp_init ? qp_init : -1);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_max", qp_max ? qp_max : 51);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_min", qp_min ? qp_min : 10);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", qp_max_i ? qp_max_i : 51);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", qp_min_i ? qp_min_i : 10);
			mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_min_i", fqp_min_i ? fqp_min_i : 10);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_max_i", fqp_max_i ? fqp_max_i : 45);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_min_p", fqp_min_p ? fqp_min_p : 10);
			mpp_enc_cfg_set_s32(cfg, "rc:fqp_max_p", fqp_max_p ? fqp_max_p : 45);
		} break;
		default : {
			spdlog::error("unsupport encoder rc mode {}", rc_mode);
		} break;
		}
	} break;
	case MPP_VIDEO_CodingVP8 : {
		/* vp8 only setup base qp range */
		mpp_enc_cfg_set_s32(cfg, "rc:qp_init", qp_init ? qp_init : 40);
		mpp_enc_cfg_set_s32(cfg, "rc:qp_max",  qp_max ? qp_max : 127);
		mpp_enc_cfg_set_s32(cfg, "rc:qp_min",  qp_min ? qp_min : 0);
		mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", qp_max_i ? qp_max_i : 127);
		mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", qp_min_i ? qp_min_i : 0);
		mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 6);
	} break;
	case MPP_VIDEO_CodingMJPEG : {
		/* jpeg use special codec config to control qtable */
		mpp_enc_cfg_set_s32(cfg, "jpeg:q_factor", qp_init ? qp_init : 80);
		mpp_enc_cfg_set_s32(cfg, "jpeg:qf_max", qp_max ? qp_max : 99);
		mpp_enc_cfg_set_s32(cfg, "jpeg:qf_min", qp_min ? qp_min : 1);
	} break;
	default : {
	} break;
	}

	/* setup codec  */
	mpp_enc_cfg_set_s32(cfg, "codec:type", encode.mpp_type);
	switch (encode.mpp_type) {
	case MPP_VIDEO_CodingAVC : {
		RK_U32 constraint_set;

		/*
		* H.264 profile_idc parameter
		* 66  - Baseline profile
		* 77  - Main profile
		* 100 - High profile
		*/
		mpp_enc_cfg_set_s32(cfg, "h264:profile", 100);
		/*
		* H.264 level_idc parameter
		* 10 / 11 / 12 / 13    - qcif@15fps / cif@7.5fps / cif@15fps / cif@30fps
		* 20 / 21 / 22         - cif@30fps / half-D1@@25fps / D1@12.5fps
		* 30 / 31 / 32         - D1@25fps / 720p@30fps / 720p@60fps
		* 40 / 41 / 42         - 1080p@30fps / 1080p@30fps / 1080p@60fps
		* 50 / 51 / 52         - 4K@30fps
		*/
		mpp_enc_cfg_set_s32(cfg, "h264:level", 40);
		mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
		mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
		mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);
	} break;
	case MPP_VIDEO_CodingHEVC : {
		mpp_enc_cfg_set_s32(cfg, "h265:diff_cu_qp_delta_depth", cu_qp_delta_depth);
	} break;
	case MPP_VIDEO_CodingMJPEG :
	case MPP_VIDEO_CodingVP8 : {
	} break;
	default : {
		spdlog::error("unsupport encoder rc mode {}", encode.mpp_type);
	} break;
	}

    // config gop_len and ref cfg
    mpp_enc_cfg_set_s32(cfg, "rc:gop", gop_len ? gop_len : fps_out_num * 2);

    if (gop_mode) {
        mpp_enc_ref_cfg_init(&ref);

        if (gop_mode < 4)
            mpi_enc_gen_ref_cfg(ref, gop_mode);
        else
            mpi_enc_gen_smart_gop_ref_cfg(ref, gop_len, vi_len);

        mpp_enc_cfg_set_ptr(cfg, "rc:ref_cfg", ref);
    }

	ret = encode.mpi->control(encode.ctx, MPP_ENC_SET_CFG, cfg);
	if (ret) {
		spdlog::error("mpi control enc set cfg failed ret {}", ret);
		goto RET;
	}

	if (ref)
		mpp_enc_ref_cfg_deinit(&ref);

	/* optional */
	ret = encode.mpi->control(encode.ctx, MPP_ENC_SET_SEI_CFG, &sei_mode);
	if (ret) {
		spdlog::error("mpi control enc set sei cfg failed ret {}", ret);
		goto RET;
	}

	if (encode.mpp_type == MPP_VIDEO_CodingAVC || encode.mpp_type == MPP_VIDEO_CodingHEVC) {
		ret = encode.mpi->control(encode.ctx, MPP_ENC_SET_HEADER_MODE, &header_mode);
		if (ret) {
			spdlog::error("mpi control enc set header mode failed ret {}", ret);
			goto RET;
		}
	}

RET:
	return ret;
}

void *__ENCODE_THREAD__(void *param)
{
	int ret;

	pthread_setname_np(pthread_self(), "__ENCODE");

	while (!encode.close) {
		ret = pthread_mutex_lock(&encode.mutex);
		assert(!ret);
		while (encode.prev_buf == NULL || !encode.ready) {
			pthread_cond_wait(&encode.cond, &encode.mutex);
			assert(!ret);
			if (encode.close) {
				ret = pthread_mutex_unlock(&encode.mutex);
				assert(!ret);
				goto end;
			}
		}

		encode.ready = false;

		int64_t start = get_cur_us();

		MppFrame frame = NULL;

		ret = mpp_frame_init(&frame);
		assert(!ret);

		mpp_frame_set_width(frame, encode.frm_width);
		mpp_frame_set_height(frame, encode.frm_height);
		mpp_frame_set_hor_stride(frame, encode.hor_stride);
		mpp_frame_set_ver_stride(frame, encode.ver_stride);
		mpp_frame_set_fmt(frame, encode.frm_fmt);
		mpp_frame_set_buffer(frame, encode.frm_buf[encode.frm_idx]);
        mpp_frame_set_eos(frame, 0);

		ret = encode.mpi->encode_put_frame(encode.ctx, frame);
		assert(!ret);

		mpp_frame_deinit(&frame);

		encode.frm_idx = (encode.frm_idx + 1) % MAX_ENCODE_BUFS;

		ret = pthread_mutex_unlock(&encode.mutex);
		assert(!ret);

		RK_U32 eoi = 1;

		do {
			MppPacket packet = NULL;

			ret = encode.mpi->encode_get_packet(encode.ctx, &packet);
			assert(!ret);
			assert(packet);

			if (mpp_packet_is_partition(packet)) {
				eoi = mpp_packet_is_eoi(packet);
			}

			if (dvr_enabled && encode.dvr != NULL) {
				void *ptr   = mpp_packet_get_pos(packet);
				size_t len  = mpp_packet_get_length(packet);

				encode.dvr->frame(std::make_shared<std::vector<uint8_t>>((uint8_t*)ptr, (uint8_t*)ptr + len));
			}

			mpp_packet_deinit(&packet);
		} while (!eoi);

		int64_t end = get_cur_us();
		spdlog::debug("Encode frame time: {} us", end - start);
	}
end:
	spdlog::info("Encode thread done.");
	return nullptr;
}
