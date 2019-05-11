#include <unistd.h>

#include "../mikes-common/bites/mikes.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/modules/live/ncurses_control.h"
#include "ui.h"
#include "../mikes-common/modules/live/gui.h"
#include "../mikes-common/modules/live/tim571.h"
#include "../mikes-common/modules/passive/x_base.h"
#include "../mikes-common/modules/passive/x_hough_tim571.h"
#include "../mikes-common/modules/live/tim_hough_transform.h"
#include "../mikes-common/modules/live/line_filter.h"
#include "../mikes-common/modules/live/tim_segment.h"
#include "../mikes-common/modules/live/tim_corner.h"
#include "../mikes-common/modules/passive/pose.h"
#include "../mikes-common/modules/passive/x_line_map.h"

#include "modules/live/rect_localization.h"
#include "modules/passive/rect_map_localization.h"

#include "core/config_mikes.h"

void init_modules()
{
  init_pose(0, 0);
  init_base_module();
  init_ncurses_control();
  init_ui();
  init_gui();

  init_tim571();
  init_tim_hough_transform();
  init_line_filter();
  init_tim_segment();
  init_tim_corner();

  init_rect_localization();
  init_rect_map_localization();

  init_x_base(400);
  init_x_tim571(7000, 400);

  init_x_line_map(mikes_config.line_map_file, 600, 350);
}

void shutdown_modules()
{
  shutdown_x_line_map();

  shutdown_rect_map_localization();
  shutdown_rect_localization();

  shutdown_tim_corner();
  shutdown_tim_segment();
  shutdown_line_filter();
  shutdown_tim_hough_transform();
  shutdown_x_tim571();

  shutdown_x_base();

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
