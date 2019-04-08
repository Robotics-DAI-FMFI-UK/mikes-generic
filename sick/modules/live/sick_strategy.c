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
#include "../../../mikes-common/bites/math_2d.h"
#include "../../../mikes-common/modules/live/avoid.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/live/nxt.h"
#include "../../../mikes-common/modules/passive/actuator.h"

#define MAX_SICK_STRATEGY_CALLBACKS 20

#define GAME_DURATION_MSEC (20*60*1000)

static pthread_mutex_t      sick_strategy_lock;
static int                  fd[2];

static sick_strategy_t        current_state;

static sick_strategy_receive_data_callback  callbacks[MAX_SICK_STRATEGY_CALLBACKS];
static int                                  callbacks_count;

static int online;

static long long time_game_started;

#define SICK_STRATEGY_LOGSTR_LEN   1024
             
static char *sick_strategy_state_str[SICK_STRATEGY_STATE__COUNT] = 
    { "?NONE", "STANDBY", "?BLOCKED", "MOVING1_TO_CART", "MOVING2_TO_CART", "WAITING_CART", "ALIGN", 
      "GRABBING", "LOADED_ESCAPE", "NOT_LOADED_ESCAPE", "RETURNING1", "RETURNING2", "RETURNING3", "RELEASING" };

void sick_strategy_log_state(int state, int state_old)
{
    char str[SICK_STRATEGY_LOGSTR_LEN];

    /* log only state changes */
    if (state == state_old) return;

    if ((state < 0) || (state >= SICK_STRATEGY_STATE__COUNT)) state = 0;
    if ((state_old < 0) || (state_old >= SICK_STRATEGY_STATE__COUNT)) state_old = 0;

    sprintf(str, "[sick_strategy] sick_strategy::sick_strategy_log_state(): state=\"%s\", state_old=\"%s\"", 
        sick_strategy_state_str[state], sick_strategy_state_str[state_old]);

    mikes_log(ML_DEBUG, str);
}

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
  uint8_t state = current_state.current;
  uint8_t state_old = current_state.old;
  pthread_mutex_unlock(&sick_strategy_lock);
  sick_strategy_log_state(state, state_old);
  printf("Changing from state %d to state %d\n", state_old, state);
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
  switch (current_state.current) {
    case SICK_STRATEGY_STATE_MOVING1_TO_CART:
      if (data->navig_result != NAVIG_RESULT_WAIT) {
        avoid_zone_enable(1, 1);
        set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING2_TO_CART);
        navig_cmd_goto_point(mikes_config.localization_waiting2_x, mikes_config.localization_waiting2_y, mikes_config.localization_waiting2_heading * RADIAN);
      }
      break;
    case SICK_STRATEGY_STATE_MOVING2_TO_CART:
      if (data->navig_result != NAVIG_RESULT_WAIT) {
        set_and_send_new_current_state(SICK_STRATEGY_STATE_ALIGN);
        align_robot_to_cart();
        /* to_debug: replace with lines below to ignore align+grabbing */
        //set_and_send_new_current_state(SICK_STRATEGY_STATE_LOADED_ESCAPE);
        //alert_new_data(fd);
      }
      break;
    case SICK_STRATEGY_STATE_RETURNING1:
      if (data->navig_result != NAVIG_RESULT_WAIT) {
        avoid_zone_enable(1, 0);
        set_and_send_new_current_state(SICK_STRATEGY_STATE_RETURNING2);
        navig_cmd_goto_point(mikes_config.localization_base2_x, mikes_config.localization_base2_y, mikes_config.localization_base2_heading * RADIAN);
      }
      break;
    case SICK_STRATEGY_STATE_RETURNING2:
      if (data->navig_result != NAVIG_RESULT_WAIT) {
        avoid_zone_enable(1, 0);
        set_and_send_new_current_state(SICK_STRATEGY_STATE_RETURNING3);
        navig_cmd_goto_point(mikes_config.localization_base3_x, mikes_config.localization_base3_y, mikes_config.localization_base3_heading * RADIAN);
      }
      break;
    case SICK_STRATEGY_STATE_RETURNING3:
      if (data->navig_result != NAVIG_RESULT_WAIT) {
        set_and_send_new_current_state(SICK_STRATEGY_STATE_RELEASING);
        alert_new_data(fd);
      }
      break;
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

      avoid_zone_enable(1, 1);
      set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING1_TO_CART);
      navig_cmd_goto_point(mikes_config.localization_waiting1_x, mikes_config.localization_waiting1_y, mikes_config.localization_waiting1_heading * RADIAN);
      break;
    case SICK_STRATEGY_STATE_LOADED_ESCAPE:
      escape_now_and_quick();

      avoid_zone_enable(1, 1);
      set_and_send_new_current_state(SICK_STRATEGY_STATE_RETURNING1);
      navig_cmd_goto_point(mikes_config.localization_base1_x, mikes_config.localization_base1_y, mikes_config.localization_base1_heading * RADIAN);
      break;
    case SICK_STRATEGY_STATE_RELEASING:
      unload_cargo();

      set_motor_speeds(8, 12);
      usleep(3500000);

      avoid_zone_enable(1, 1);
      set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING1_TO_CART);
      navig_cmd_goto_point(mikes_config.localization_waiting1_x, mikes_config.localization_waiting1_y, mikes_config.localization_waiting1_heading * RADIAN);
      break;
    case SICK_STRATEGY_STATE_STANDBY:
      // waiting until calling start_game()
      break;
    default: 
      {
        char str[SICK_STRATEGY_LOGSTR_LEN];
        sprintf(str, "[sick_strategy] sick_strategy::process_next_step(): msg=\"wrong state!\", state=%d", current_state.current);
        mikes_log(ML_ERR, str);
        printf("Process next step with not defined behavior %d\n", current_state.current);
        return;
      }
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
    say("Lets go!");
    avoid_zone_enable(1, 1);
    set_and_send_new_current_state(SICK_STRATEGY_STATE_MOVING1_TO_CART);
    navig_cmd_goto_point(mikes_config.localization_waiting1_x, mikes_config.localization_waiting1_y, mikes_config.localization_waiting1_heading * RADIAN);
    time_game_started = msec();
  }
}

int game_timeout()
{
  return msec() - time_game_started > GAME_DURATION_MSEC;
}
