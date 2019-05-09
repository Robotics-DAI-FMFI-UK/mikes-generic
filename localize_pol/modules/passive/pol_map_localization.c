#include <pthread.h>
#include <unistd.h>
#include <stdio.h>

#include "core/config_mikes.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"

#include "../../../mikes-common/modules/passive/x_line_map.h"
#include "../../../mikes-common/modules/passive/pose.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/live/tim_hough_transform.h"

#include "../live/pol_localization.h"

void update_pol_localization(pol_localization_t *result)
{
  if (result->status == POL_LOCALIZATION_SUCCESS) {
    pose_type copy_p;
    copy_p.x = result->pose.x;
    copy_p.y = result->pose.y;
    copy_p.heading = result->pose.heading;

    set_pose(copy_p.x, copy_p.y, copy_p.heading);
    x_line_map_toggle_pose_visible(1);
  }
}

void update_base_data(base_data_type *data)
{
  pose_type p;
  get_pose(&p);
  p.x = p.x + 11;
  p.y = p.y + 11;
  x_line_map_set_pose(p);
}

void init_pol_map_localization()
{
  register_base_callback(update_base_data);
  register_pol_localization_callback(update_pol_localization);
}

void shutdown_pol_map_localization()
{
  unregister_pol_localization_callback(update_pol_localization);
  unregister_base_callback(update_base_data);
}
