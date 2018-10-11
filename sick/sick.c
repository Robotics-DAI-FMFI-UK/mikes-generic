#include <unistd.h>
#include <stdio.h>

#include "../mikes-common/bites/mikes.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/modules/live/ncurses_control.h"
#include "ui.h"
#include "../mikes-common/modules/live/gui.h"
#include "../mikes-common/modules/live/tim571.h"
#include "../mikes-common/modules/live/tim_hough_transform.h"
#include "../mikes-common/modules/live/line_filter.h"
#include "../mikes-common/modules/live/tim_segment.h"
#include "../mikes-common/modules/live/tim_corner.h"
//#include "../mikes-common/modules/live/xtion/xtion.h"
#include "../mikes-common/modules/live/avoid.h"
#include "../mikes-common/modules/live/navig.h"
#include "../mikes-common/modules/passive/x_base.h"
#include "../mikes-common/modules/passive/x_tim571.h"
//#include "../mikes-common/modules/passive/x_xtion.h"
#include "../mikes-common/modules/passive/x_line_map.h"
#include "../mikes-common/modules/passive/actuator.h"
#include "../mikes-common/modules/passive/wheels.h"
#include "../mikes-common/modules/live/nxt.h"

#include "core/config_mikes.h"
#include "modules/live/sick_localization.h"
#include "modules/live/sick_cart_align.h"
#include "modules/live/sick_strategy.h"
#include "modules/passive/sick_map_localize.h"

void init_modules()
{
  init_base_module();
  set_motor_speeds(0, 0);
  init_ncurses_control();
  init_ui();
  init_gui();
  init_tim571();
  init_tim_hough_transform();
  init_line_filter();
  init_tim_segment();
  init_tim_corner();
  init_avoid();
  init_navig();
  init_nxt();
  init_wheels();

  init_x_base(400);
  init_x_tim571(7000, 400);

  init_x_line_map(mikes_config.line_map_file, 600, 350);

  init_sick_localization();
  init_sick_map_localize();
  init_sick_cart_align();
  init_sick_strategy();
  sleep(3);
  init_actuator();
}

void shutdown_modules()
{
  shutdown_sick_strategy();
  shutdown_sick_cart_align();
  shutdown_sick_map_localize();
  shutdown_sick_localization();
  shutdown_tim_hough_transform();
  shutdown_line_filter();
  shutdown_tim_segment();
  shutdown_tim_corner();

  shutdown_x_line_map();
  shutdown_x_tim571();
  shutdown_x_base();
  shutdown_nxt();
  shutdown_wheels();
  shutdown_avoid();
  shutdown_navig();
  shutdown_gui();
  shutdown_ui();
}

void sick_main_menu()
{
  nxt_clear_display();
  nxt_message("Mikes menu");
  nxt_message("[ENTER] run");
  nxt_message("[<] quit");
  int key = nxt_read_key();
  if (key == NXT_KEY_ENTER) start_game();
  else if (key == NXT_KEY_LEFT) program_runs = 0;
}

int main(int argc, char **argv)
{
  mikes_init(argc, argv);
  init_modules();

  use_config_initial_localization();
  find_starting_localization();

  start_game();

  while (program_runs)
  {
     sleep(1);
  }
  printf("going to shutdown modules\n");

  shutdown_modules();
  mikes_shutdown();
  return 0;
}
