#include <stdio.h>

#include "../mikes-common/modules/live/ncurses_control.h"
#include "../mikes-common/modules/live/base_module.h"
#include "../mikes-common/bites/mikes.h"

static int window;

void key_listener(int key)
{
  char prn[100];
  sprintf(prn, "key=%d (%c)", key, key);
  println_to_window(window, prn);

  switch (key) {
    case KEY_UP: set_motor_speeds(12, 12);
                 break;
    case KEY_LEFT: set_motor_speeds(-12, 12);
                   break;
    case KEY_RIGHT: set_motor_speeds(12, -12);
                    break;
    case KEY_DOWN: set_motor_speeds(-12, -12);
                   break;
    case ' ': stop_now();
              break;
    case KEY_ESC: program_runs = 0;
                  break;
  }
}

void init_ui()
{
   window = open_window();
   add_key_listener("manual", key_listener);
}

void shutdown_ui()
{
  remove_key_listener("manual");
  close_window(window);
}

