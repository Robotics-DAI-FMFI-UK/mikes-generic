#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "sick_strategy.h"
#include "../../../mikes-common/bites/mikes.h"
#include "../../../mikes-common/bites/util.h"
#include "../../../mikes-common/bites/math_2d.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "core/config_mikes.h"

#include "sick_cart_align.h"
#include "../../../mikes-common/modules/live/avoid.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/live/nxt.h"
#include "../../../mikes-common/modules/passive/actuator.h"

#define SICK_STRATEGY_WAITING_POINT_X 300.0 // TODO in CM
#define SICK_STRATEGY_WAITING_POINT_Y 250.0 // TODO in CM
#define SICK_STRATEGY_WAITING_POINT_HEADING 300.0 * RADIAN// TODO

#define SICK_STRATEGY_RETURNING_POINT_X 550.0 // TODO in CM
#define SICK_STRATEGY_RETURNING_POINT_Y 170.0 // TODO in CM
#define SICK_STRATEGY_RETURNING_POINT_HEADING 270.0 * RADIAN// TODO

#define MAX_SICK_STRATEGY_CALLBACKS 20

#define GAME_DURATION_MSEC (10*60*1000)

static pthread_mutex_t      sick_strategy_lock;
static int                  fd[2];

static sick_strategy_t        current_state;

static sick_strategy_receive_data_callback  callbacks[MAX_SICK_STRATEGY_CALLBACKS];
static int                                  callbacks_count;

static int online;

static long long time_game_started;

void create_copy_of_current_state(sick_strategy_t *copy)
{
  pthread_mutex_lock(&sick_strategy_lock);
  *copy = current_state;
  pthread_mutex_unlock(&sick_strategy_lock);
}

void set_and_send_new_current_state(uint8_t new_state)
{
  pthread_mutex_lock(&sick_strategy_lock);
  current_state.old = current_state.current;
  current_state.current = new_state;
  for (int i = 0; i < callbacks_count; i++) {
    callbacks[i](&current_state);
  }
  pthread_mutex_unlock(&sick_strategy_lock);
}

void update_sick_cart_align_callback(sick_cart_align_t *result)
{
  if (current_state.current == SICK_STRATEGY_STATE_ALIGN)
  {
    if (result->status == SICK_CART_ALIGN_SUCCESS)
    {
       set_and_send_new_current_state(SICK_STRATEGY_STATE_GRABBING);
       alert_new_data(fd);
    }
    else
    {
       set_and_send_new_current_state(SICK_STRATEGY_STATE_NOT_LOADED_ESCAPE);
       alert_new_data(fd);
    }
  }
}

void update_avoid_callback(avoid_callback_data_t *data)
{
  // TODO
}

void update_navig_callback(navig_callback_data_t *data)
{
  if (current_state.current == SICK_STRATEGY_STATE_MOVING_TO_CART) {
    switch (data->navig_result) {
      case NAVIG_RESULT_OK:
        set_and_send_new_current_state(SICK_STRATEGY_STATE_ALIGN);
        align_robot_to_cart();
        break;
      case NAVIG_RESULT_FAILED:
        // TODO do smth
        break;
      case NAVIG_RESULT_WAIT:
        break;
    }
  } else if (current_state.current == SICK_STRATEGY_STATE_RETURNING) {
    switch (data->navig_result) {
      case NAVIG_RESULT_OK:
        set_and_send_new_current_state(SICK_STRATEGY_STATE_RELEASING);
        alert_new_data(fd);
        break;
      case NAVIG_RESULT_FAILED:
        // TODO do smth
        break;
      case NAVIG_RESULT_WAIT:
        break;
    }
  }
}

void process_next_step()
{
  switch (current_state.current) {
    case SICK_STRATEGY_STATE_GRABBING:
      grab_line(1);
      grab_line(2);
      grab_line(3);

      set_and_send_new_current_state(SICK_STRATEGY_STATE_LOADED_ESCAPE);
      alert_new_data(fd);
      break;
    case SICK_STRATEGY_STATE_NOT_LOADED_ESCAPE:
      escape_now_and_quick();

      set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING_TO_CART);
      navig_cmd_goto_point(SICK_STRATEGY_WAITING_POINT_X, SICK_STRATEGY_WAITING_POINT_Y, SICK_STRATEGY_WAITING_POINT_HEADING);
      break;
    case SICK_STRATEGY_STATE_LOADED_ESCAPE:
      escape_now_and_quick();

      set_and_send_new_current_state(SICK_STRATEGY_STATE_RETURNING);
      navig_cmd_goto_point(SICK_STRATEGY_RETURNING_POINT_X, SICK_STRATEGY_RETURNING_POINT_Y, SICK_STRATEGY_RETURNING_POINT_HEADING);
      break;
    case SICK_STRATEGY_STATE_RELEASING:
      unload_cargo();

      set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING_TO_CART);
      navig_cmd_goto_point(SICK_STRATEGY_WAITING_POINT_X, SICK_STRATEGY_WAITING_POINT_Y, SICK_STRATEGY_WAITING_POINT_HEADING);
      break;
    case SICK_STRATEGY_STATE_STANDBY:
      // TODO maybe can do smth hre, atleast log
      break;
    default:
      printf("Process next step with not defined behavior %d\n", current_state.current);
      return;
  }
}

void *sick_strategy_thread(void *args)
{
  while (program_runs)
  {
    if (wait_for_new_data(fd) < 0) {
      perror("mikes:sick_strategy");
      mikes_log(ML_ERR, "sick_strategy error during waiting on new Data.");
      continue;
    }

    if (game_timeout())
       set_and_send_new_current_state(SICK_STRATEGY_STATE_STANDBY);

    process_next_step();
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
  current_state.current = SICK_STRATEGY_STATE_STANDBY;

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

void start_game()
{
  if (current_state.current == SICK_STRATEGY_STATE_STANDBY)
  {
    set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING_TO_CART);
    navig_cmd_goto_point(SICK_STRATEGY_WAITING_POINT_X, SICK_STRATEGY_WAITING_POINT_Y, SICK_STRATEGY_WAITING_POINT_HEADING);
    say("Let's go!");
    time_game_started = msec();
  }
}

int game_timeout()
{
  return msec() - time_game_started > GAME_DURATION_MSEC;
}
