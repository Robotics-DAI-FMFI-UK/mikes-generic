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

#define MAX_SICK_STRATEGY_CALLBACKS 20

static pthread_mutex_t      sick_strategy_lock;
static int                  fd[2];

static sick_localization_receive_data_callback  callbacks[MAX_SICK_STRATEGY_CALLBACKS];
static int                                      callbacks_count;

static int online;

void process_next_step()
{
  // TODO
}

void *sick_strategy_thread(void *args)
{
  while (program_runs)
  {
    // TODO make some switch on state instead maybe
    if (wait_for_new_data(fd) < 0) {
      perror("mikes:sick_strategy");
      mikes_log(ML_ERR, "sick_strategy error during waiting on new Data.");
      continue;
    }
    pthread_mutex_lock(&sick_strategy_lock);
    process_next_step();
    pthread_mutex_unlock(&sick_strategy_lock);
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

  if (pipe(fd) != 0)
  {
    perror("mikes:sick_strategy");
    mikes_log(ML_ERR, "creating pipe for sick strategy");
    return;
  }

  pthread_t t;
  pthread_mutex_init(&sick_strategy_lock, 0);
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
