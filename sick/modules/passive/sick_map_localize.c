#include <stdio.h>
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "../../../mikes-common/modules/passive/x_line_map.h"
#include "../../../mikes-common/modules/passive/pose.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"

#include "../live/sick_localization.h"

static int was_set = 0;

#define SML_LOGSTR_LEN 1024

void update_sick_localization(pose_type *pose)
{
  if (!was_set) {
    x_line_map_toggle_pose_visible(1);
    was_set = 1;
  }

  pose_type copy_p;
  copy_p.x = pose->x / 10.0 + 11;
  copy_p.y = pose->y / 10.0 + 11;
  copy_p.heading = pose->heading;

  pose_type old_p;
  get_pose(&old_p);

  base_data_type base_data;
  get_base_data(&base_data);

  char str[SML_LOGSTR_LEN];

  if (navig_can_actualize_pose_now()) {
    x_line_map_set_pose(copy_p);
    set_pose(copy_p.x, copy_p.y, copy_p.heading);

    sprintf(str, "[main] sick_map_localize::update_sick_localization_updated(): x=%0.2f, y=%0.2f, heading_deg=%0.2f, old_x=%0.2f, old_y=%0.2f, old_heading_deg=%0.2f, base_heading=%d",
      copy_p.x, copy_p.y, copy_p.heading / M_PI * 180.0, old_p.x, old_p.y, old_p.heading / M_PI * 180.0, (int)base_data.heading);
    mikes_log(ML_DEBUG, str);
  } else {
    sprintf(str, "[main] sick_map_localize::update_sick_localization_not_updated(): x=%0.2f, y=%0.2f, heading_deg=%0.2f, old_x=%0.2f, old_y=%0.2f, old_heading_deg=%0.2f, base_heading=%d",
      copy_p.x, copy_p.y, copy_p.heading / M_PI * 180.0, old_p.x, old_p.y, old_p.heading / M_PI * 180.0, (int)base_data.heading);
    mikes_log(ML_DEBUG, str);
  }
}

void update_base_data(base_data_type *data)
{
  pose_type p;
  get_pose(&p);
//  x_line_map_set_pose(p);
}

void init_sick_map_localize()
{
  register_base_callback(update_base_data);
  register_sick_localization_callback(update_sick_localization);
}

void shutdown_sick_map_localize()
{
  unregister_sick_localization_callback(update_sick_localization);
  unregister_base_callback(update_base_data);
}
