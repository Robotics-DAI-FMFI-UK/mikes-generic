#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "../mikes-common/modules/live/ncurses_control.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/bites/mikes.h"
#include "../mikes-common/modules/live/gridmapping.h"
#include "../mikes-common/modules/live/mapping_navig.h"

static int window;

void key_listener(int key)
{
  char prn[100];
  sprintf(prn, "key=%d (%c)", key, key);
  println_to_window(window, prn);

  switch (key) {
    case KEY_UP_ARROW: set_motor_speeds(12, 12);
                       break;
    case KEY_LEFT_ARROW: set_motor_speeds(-12, 12);
                         break;
    case KEY_RIGHT_ARROW: set_motor_speeds(12, -12);
                          break;
    case KEY_DOWN_ARROW: set_motor_speeds(-12, -12);
                         break;
    case ' ': stop_now();
              pause_mapping_navig(1);
              usleep(1500000L);
              start_scanning();
              break;
    case 'p': pause_mapping_navig(1);
    case 's': pause_mapping_navig(0);
    case KEY_ESC: program_runs = 0;
                  break;
  }
}

void init_ui()
{
   window = open_window();
   set_window_title(window, "pressed keys");
   add_key_listener("manual", key_listener);
}

void shutdown_ui()
{
  remove_key_listener("manual");
  close_window(window);
}

