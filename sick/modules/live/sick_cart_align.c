#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "sick_cart_align.h"
#include "../../../mikes-common/bites/mikes.h"
#include "../../../mikes-common/bites/util.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "core/config_mikes.h"

#define MAX_SICK_CART_ALIGN_CALLBACKS 20

static pthread_mutex_t      sick_cart_align_lock;
static int                  fd[2];

sick_cart_align_t           result;

static sick_cart_align_receive_data_callback  callbacks[MAX_SICK_CART_ALIGN_CALLBACKS];
static int                                    callbacks_count;

static int online;

uint8_t process_align_cart()
{
  return SICK_CART_ALIGN_SUCCESS;
}

void process_new_request()
{
  result.status = process_align_cart();
  for (int i = 0; i < callbacks_count; i++) {
    callbacks[i](&result);
  }
}

void *sick_cart_align_thread(void *args)
{
  while (program_runs)
  {
    if (wait_for_new_data(fd) < 0) {
      perror("mikes:sick_cart_align");
      mikes_log(ML_ERR, "sick_cart_align error during waiting on new Data.");
      continue;
    }
    pthread_mutex_lock(&sick_cart_align_lock);
    process_new_request();
    pthread_mutex_unlock(&sick_cart_align_lock);
  }

  mikes_log(ML_INFO, "sick_cart_align quits.");
  threads_running_add(-1);
  return 0;
}

void init_sick_cart_align()
{
  if (!mikes_config.use_sick_cart_align)
  {
    mikes_log(ML_INFO, "sick_cart_align supressed by config.");
    online = 0;
    return;
  }
  online = 1;

  if (pipe(fd) != 0)
  {
    perror("mikes:sick_cart_align");
    mikes_log(ML_ERR, "creating pipe for sick cart align");
    return;
  }

  pthread_t t;
  pthread_mutex_init(&sick_cart_align_lock, 0);
  if (pthread_create(&t, 0, sick_cart_align_thread, 0) != 0)
  {
    perror("mikes:sick_cart_align");
    mikes_log(ML_ERR, "creating thread for sick cart align");
  }
  else threads_running_add(1);
}

void shutdown_sick_cart_align()
{
  online = 0;

  close(fd[0]);
  close(fd[1]);
}

void align_robot_to_cart()
{
  if (!online) return;

  if (pthread_mutex_trylock(&sick_cart_align_lock) == 0) {
    alert_new_data(fd);
    pthread_mutex_unlock(&sick_cart_align_lock);
  }
}

void register_sick_cart_align_callback(sick_cart_align_receive_data_callback callback)
{
  if (!online) return;

  if (callbacks_count >= MAX_SICK_CART_ALIGN_CALLBACKS)
  {
     mikes_log(ML_ERR, "too many sick_localization callbacks");
     return;
  }
  callbacks[callbacks_count++] = callback;
}

void unregister_sick_cart_align_callback(sick_cart_align_receive_data_callback callback)
{
  if (!online) return;

  for (int i = 0; i < callbacks_count; i++) {
    if (callbacks[i] == callback) {
       callbacks[i] = callbacks[(callbacks_count--) - 1];
    }
  }
}
