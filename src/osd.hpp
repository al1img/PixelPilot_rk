#ifndef OSDPP_H
#define OSDPP_H

extern "C" {
#include "osd.h"
}
#include <nlohmann/json.hpp>

typedef struct {
	modeset_bufs *osd_bufs;
	int fd;
	nlohmann::json config;
} osd_thread_params;

extern int osd_thread_signal;

struct SharedMemoryRegion {
    uint16_t width;       // Image width
    uint16_t height;      // Image height
    unsigned char data[]; // Flexible array member for image data
};

void *__OSD_THREAD__(void *param);

#endif
