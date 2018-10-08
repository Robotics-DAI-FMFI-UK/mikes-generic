#ifndef _SICK_LOCALIZATION_H_
#define _SICK_LOCALIZATION_H_

#define SICK_MAP_WITH_IN_MM 5800
#define SICK_MAP_HEIGHT_IN_MM 3270

#include "../../../mikes-common/modules/passive/pose.h"

typedef void (*sick_localization_receive_data_callback)(pose_type *pose);

void init_sick_localization();
void shutdown_sick_localization();

void register_sick_localization_callback(sick_localization_receive_data_callback callback);
void unregister_sick_localization_callback(sick_localization_receive_data_callback callback);

#endif
