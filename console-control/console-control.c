#include <unistd.h>

#include "../mikes-common/bites/mikes.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/modules/live/ncurses_control.h"
#include "ui.h"

int main(int argc, char **argv)
{
  mikes_init(argc, argv);
  init_base_module();
  init_ncurses_control();
  init_ui();
  
  while (program_runs)
  {
     sleep(1);
  }

  shutdown_ui();
  mikes_shutdown();
  return 0;
}

