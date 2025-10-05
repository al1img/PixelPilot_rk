// encode.h
#ifndef ENCODE_H
#define ENCODE_H

#include <pthread.h>

#include <rockchip/mpp_frame.h>
#include <rockchip/rk_mpi.h>

#include "drm.h"
#include "dvr.h"

/***********************************************************************************************************************
 * Consts
 **********************************************************************************************************************/

#define MAX_ENCODE_BUFS 2

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/

#define CODEC_ALIGN(x, a)   (((x)+(a)-1)&~((a)-1))

/***********************************************************************************************************************
 * Types
 **********************************************************************************************************************/

struct encode_ctx {
	bool enable;
	bool ready;
    bool close;
	MppBufferGroup frm_grp;
	MppBuffer frm_buf[MAX_ENCODE_BUFS];
	int frm_idx;
	MppFrameFormat frm_fmt;
	uint32_t frm_width;
	uint32_t frm_height;
	uint32_t hor_stride;
	uint32_t ver_stride;
	int frame_rate;
	MppBuffer osd_buf;
	MppBuffer cur_buf;
	MppBuffer prev_buf;
	MppCodingType mpp_type;
	MppCtx ctx;
	MppApi *mpi;
    pthread_t tid;
    pthread_mutex_t mutex;
	pthread_cond_t cond;
    Dvr *dvr;
};

/***********************************************************************************************************************
 * Vars
 **********************************************************************************************************************/

extern encode_ctx encode;

/***********************************************************************************************************************
 * Public functions
 **********************************************************************************************************************/

void init_encode(Dvr *dvr, MppCodingType mpp_type, int frame_rate);
void release_encode();

void create_encode_buffers(uint32_t width, uint32_t height, uint32_t hstride, uint32_t vstride, MppFrameFormat fmt);
void release_encode_buffers();

void update_encode_buffer(MppBuffer buffer);
void process_encode();

int copy_osd_buf(const modeset_buf& osd_buf);

#endif
