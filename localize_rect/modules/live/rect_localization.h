#ifndef _RECT_LOCALIZATION_H_
#define _RECT_LOCALIZATION_H_

#include <stdint.h>

#include "../../../mikes-common/modules/passive/pose.h"

#define RECT_MAP_WITH_IN_MM    5800
#define RECT_MAP_HEIGHT_IN_MM  3220

#define RECT_LOCALIZATION_FAIL    0
#define RECT_LOCALIZATION_SUCCESS 1

typedef struct {
    uint8_t status;
    pose_type pose;
} rect_localization_t;

typedef void (*rect_localization_receive_data_callback)(rect_localization_t *result);

void init_rect_localization();
void shutdown_rect_localization();

void enable_rect_localization(int enable);

void register_rect_localization_callback(rect_localization_receive_data_callback callback);
void unregister_rect_localization_callback(rect_localization_receive_data_callback callback);

#endif
