#include <unistd.h>

#include "../mikes-common/bites/mikes.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/modules/live/ncurses_control.h"
#include "ui.h"
#include "../mikes-common/modules/live/gui.h"
#include "../mikes-common/modules/live/lidar.h"
#include "../mikes-common/modules/live/ust10lx.h"
#include "../mikes-common/modules/live/tim571.h"
#include "../mikes-common/modules/live/tim_hough_transform.h"
#include "../mikes-common/modules/live/line_filter.h"
#include "../mikes-common/modules/live/tim_segment.h"
#include "../mikes-common/modules/live/tim_corner.h"
#include "../mikes-common/modules/live/xtion/xtion.h"
#include "../mikes-common/modules/live/rfid_sensor.h"
#include "../mikes-common/modules/passive/x_base.h"
#include "../mikes-common/modules/passive/x_lidar.h"
#include "../mikes-common/modules/passive/x_ust10lx.h"
#include "../mikes-common/modules/passive/x_tim571.h"
#include "../mikes-common/modules/passive/x_xtion.h"
#include "../mikes-common/modules/passive/x_line_map.h"

#include "core/config_mikes.h"
#include "modules/live/sick_localization.h"

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

void init_modules()
{
  init_base_module();
  init_ncurses_control();
  init_ui();
  init_gui();

  init_lidar();
  init_ust10lx();
  init_tim571();
  init_tim_hough_transform();
  init_line_filter();
  init_tim_segment();
  init_tim_corner();
  init_xtion(64, 48);
  init_rfid_sensor();

  init_sick_localization();

  init_x_base(400);
  init_x_lidar(7000, 400);
  init_x_ust10lx(7000, 400);
  init_x_tim571(7000, 400);
  init_x_xtion(64, 48, 4, 300);

  init_x_line_map(mikes_config.line_map_file, 600, 350);

  register_sick_localization_callback(update_sick_localization);
}

void shutdown_modules()
{
  shutdown_x_line_map();
  shutdown_x_xtion();
  shutdown_x_tim571();
  shutdown_x_ust10lx();
  shutdown_x_lidar();
  shutdown_x_base();

  shutdown_sick_localization();

  shutdown_gui();
  shutdown_ui();
}

int main(int argc, char **argv)
{
  mikes_init(argc, argv);
  init_modules();

  while (program_runs)
  {
     sleep(1);
  }

  shutdown_modules();
  mikes_shutdown();
  return 0;
}
