#include "../../../mikes-common/modules/passive/x_line_map.h"

#include "../live/sick_localization.h"

static int was_set = 0;

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
  x_line_map_set_pose(copy_p);
}

void init_sick_map_localize()
{
  register_sick_localization_callback(update_sick_localization);
}

void shutdown_sick_map_localize()
{
  unregister_sick_localization_callback(update_sick_localization);
}
