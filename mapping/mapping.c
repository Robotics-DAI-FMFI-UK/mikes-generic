#include <unistd.h>

#include "../mikes-common/bites/mikes.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/modules/live/ncurses_control.h"
#include "ui.h"
#include "../mikes-common/modules/live/gui.h"
#include "../mikes-common/modules/live/tim571.h"
#include "../mikes-common/modules/live/t265.h"
#include "../mikes-common/modules/live/xtion/xtion.h"
#include "../mikes-common/modules/live/gridmapping.h"
#include "../mikes-common/modules/passive/gridmap.h"
#include "../mikes-common/modules/passive/x_base.h"
#include "../mikes-common/modules/passive/pose.h"
#include "../mikes-common/modules/passive/x_tim571.h"
#include "../mikes-common/modules/passive/x_xtion.h"
#include "../mikes-common/modules/passive/x_gridmap.h"

#include "core/config_mikes.h"

void init_modules()
{
  init_pose(0, 0);
  init_base_module();
  init_ncurses_control();
  init_ui();
  init_gui();
  
  init_tim571();
  init_t265();
  init_gridmap();
  init_gridmapping();
  init_xtion(64, 48);

  init_x_base(400);
  init_x_tim571(7000, 400);
  init_x_xtion(64, 48, 4, 300);

  init_x_gridmap(440, 440, 400);
}

void shutdown_modules()
{
  shutdown_gridmapping();
  shutdown_x_gridmap();
  shutdown_x_xtion();
  shutdown_x_tim571();
  shutdown_x_base();

  shutdown_gui();
  shutdown_ui();
}

int main(int argc, char **argv)
{
  mikes_init(argc, argv);
  init_modules();
  x_gridmap_toggle_pose_visible(1);
  
  while (program_runs)
  {
     sleep(1);
  }

  shutdown_modules();
  mikes_shutdown();
  return 0;
}

