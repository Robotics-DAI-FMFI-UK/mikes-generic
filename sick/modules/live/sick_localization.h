#ifndef _SICK_LOCALIZATION_H_
#define _SICK_LOCALIZATION_H_

#include <stdint.h>

#include "../../../mikes-common/modules/passive/pose.h"

#define SICK_MAP_WITH_IN_MM    13230 /* 5800 */
#define SICK_MAP_HEIGHT_IN_MM   7090 /* 3270 */

#define SICK_LOCALIZATION_FAIL    0
#define SICK_LOCALIZATION_SUCCESS 1

typedef struct {
    uint8_t status;
    pose_type pose;
} sick_localization_t;

typedef void (*sick_localization_receive_data_callback)(sick_localization_t *result);

void init_sick_localization();
void shutdown_sick_localization();

void register_sick_localization_callback(sick_localization_receive_data_callback callback);
void unregister_sick_localization_callback(sick_localization_receive_data_callback callback);

#endif
