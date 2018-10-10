#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "sick_strategy.h"
#include "../../../mikes-common/bites/mikes.h"
#include "../../../mikes-common/bites/util.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "core/config_mikes.h"

#include "sick_cart_align.h"
#include "../../../mikes-common/modules/live/avoid.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/passive/actuator.h"

#define SICK_STRATEGY_WAITING_POINT_X 3000.0
#define SICK_STRATEGY_WAITING_POINT_Y 3000.0
#define SICK_STRATEGY_WAITING_POINT_HEADING 300.0

#define MAX_SICK_STRATEGY_CALLBACKS 20

static pthread_mutex_t      sick_strategy_lock;
static int                  fd[2];

static sick_strategy_t current_state;

static sick_strategy_receive_data_callback  callbacks[MAX_SICK_STRATEGY_CALLBACKS];
static int                                  callbacks_count;

static int online;

void update_sick_cart_align_callback(sick_cart_align_t *result)
{
  // TODO
}

void update_avoid_callback(avoid_callback_data_t *data)
{
  // TODO
}

void update_navig_callback(navig_callback_data_t *data)
{
  // TODO
}

void set_new_current_state(uint8_t new_state)
{
  current_state.old = current_state.current;
  current_state.current = new_state;
}

void send_current_state()
{
  for (int i = 0; i < callbacks_count; i++) {
    callbacks[i](&current_state);
  }
}

void process_next_step()
{
  // TODO
  switch (current_state.current) {
    case SICK_STRATEGY_STATE_INITIAL:
      set_new_current_state(SICK_STRATEGY_STATE_MOVING_TO_CART);
      send_current_state();
      navig_cmd_goto_point(SICK_STRATEGY_WAITING_POINT_X, SICK_STRATEGY_WAITING_POINT_Y, SICK_STRATEGY_WAITING_POINT_HEADING);
      break;
    case SICK_STRATEGY_STATE_BLOCKED:
      break;
    case SICK_STRATEGY_STATE_MOVING_TO_CART:
      break;
    case SICK_STRATEGY_STATE_WAITING_CART:
      break;
    case SICK_STRATEGY_STATE_ALIGN:
      break;
    case SICK_STRATEGY_STATE_GRABBING:
      break;
    case SICK_STRATEGY_STATE_RETURNING:
      break;
    case SICK_STRATEGY_STATE_RELEASING:
      break;
    default:
      printf("Unknown sick strategy step %d\n", current_state.current);
      return;
  }

  send_current_state();
}

void *sick_strategy_thread(void *args)
{
  uint8_t was_read_error = 0;

  while (program_runs)
  {
    if (was_read_error) {
      was_read_error = 0;
    } else {
      pthread_mutex_lock(&sick_strategy_lock);
      process_next_step();
      pthread_mutex_unlock(&sick_strategy_lock);
    }

    if (wait_for_new_data(fd) < 0) {
      was_read_error = 1;
      perror("mikes:sick_strategy");
      mikes_log(ML_ERR, "sick_strategy error during waiting on new Data.");
      continue;
    }
  }

  mikes_log(ML_INFO, "sick_strategy quits.");
  threads_running_add(-1);
  return 0;
}

void init_sick_strategy()
{
  if (!mikes_config.use_sick_strategy)
  {
    mikes_log(ML_INFO, "sick_strategy supressed by config.");
    online = 0;
    return;
  }
  online = 1;
  current_state.current = SICK_STRATEGY_STATE_INITIAL;

  if (pipe(fd) != 0)
  {
    perror("mikes:sick_strategy");
    mikes_log(ML_ERR, "creating pipe for sick strategy");
    return;
  }

  pthread_t t;
  pthread_mutex_init(&sick_strategy_lock, 0);
  register_sick_cart_align_callback(update_sick_cart_align_callback);
  avoid_register_callback(update_avoid_callback);
  navig_register_callback(update_navig_callback);
  if (pthread_create(&t, 0, sick_strategy_thread, 0) != 0)
  {
    perror("mikes:sick_strategy");
    mikes_log(ML_ERR, "creating thread for sick strategy");
  }
  else threads_running_add(1);
}

void shutdown_sick_strategy()
{
  online = 0;

  close(fd[0]);
  close(fd[1]);
}

void register_sick_strategy_callback(sick_strategy_receive_data_callback callback)
{
  if (!online) return;

  if (callbacks_count >= MAX_SICK_STRATEGY_CALLBACKS)
  {
     mikes_log(ML_ERR, "too many sick_strategy callbacks");
     return;
  }
  callbacks[callbacks_count++] = callback;
}

void unregister_sick_strategy_callback(sick_strategy_receive_data_callback callback)
{
  if (!online) return;

  for (int i = 0; i < callbacks_count; i++) {
    if (callbacks[i] == callback) {
       callbacks[i] = callbacks[(callbacks_count--) - 1];
    }
  }
}
