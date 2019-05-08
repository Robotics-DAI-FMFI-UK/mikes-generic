#ifndef _POL_LOCALIZATION_H_
#define _POL_LOCALIZATION_H_

#include <stdint.h>

#include "../../../mikes-common/modules/passive/pose.h"

#define POL_LOCALIZATION_FAIL    0
#define POL_LOCALIZATION_SUCCESS 1

#define MAX_POL_SEGEMNTS 20

typedef struct {
    segment_data segment;
    corner_data corner1;
    corner_data corner2;
} pol_segment_t;

typedef struct {
    pol_segment_t segments[MAX_POL_SEGEMNTS];
    int count;
} pol_segments_t;

typedef struct {
    uint8_t status;
    pose_type pose;
} pol_localization_t;

typedef void (*pol_localization_receive_data_callback)(pol_localization_t *result);

void init_pol_localization();
void shutdown_pol_localization();

void register_pol_localization_callback(pol_localization_receive_data_callback callback);
void unregister_pol_localization_callback(pol_localization_receive_data_callback callback);

#endif
