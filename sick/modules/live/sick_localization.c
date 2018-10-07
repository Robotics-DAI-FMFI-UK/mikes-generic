#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "../../../mikes-common/bites/mikes.h"
#include "../../../mikes-common/bites/util.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "../../../mikes-common/modules/live/tim_corner.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "core/config_mikes.h"

#define MAX_SICK_LOCALIZATION_CALLBACKS 20

static pthread_mutex_t      sick_localization_lock;
static int                  fd[2];

static corners_data         corners_local_copy;
static base_data_type       base_data_local_copy;

static pose_type            pose_localization_local;

static sick_localization_receive_data_callback  callbacks[MAX_SICK_LOCALIZATION_CALLBACKS];
static int                                      callbacks_count;

static int online;

// ----------------------------------------------------------------
// ----------------------------------------------------------------
// --------------------------LIFECYCLE-----------------------------
// ----------------------------------------------------------------
// ----------------------------------------------------------------

void get_pose_base_on_corners_and_heading(corners_data *corners, base_data_type *base_data, pose_type *result_pose)
{
  corner_print_data(corners);
  printf("%lu: LCNT:%ld RCNT:%ld LVEL:%d RVEL:%d IR:(%d,%d,%d,%d) HEAD:%d ACC:(%d,%d,%d) GYR:(%d,%d,%d)\n",
         base_data->timestamp, base_data->counterA, base_data->counterB, base_data->velocityA, base_data->velocityB,
         base_data->dist1, base_data->dist2, base_data->dist3, base_data->cube, base_data->heading,
         base_data->ax, base_data->ay, base_data->az, base_data->gx, base_data->gy, base_data->gz);
}

void process_all_data()
{
  get_pose_base_on_corners_and_heading(&corners_local_copy, &pose_local_copy, &pose_localization_local);
  for (int i = 0; i < callbacks_count; i++)
    callbacks[i](&pose_localization_local);
}

void *sick_localization_thread(void *args)
{
  while (program_runs)
  {
    if (wait_for_new_data(fd) < 0) {
      perror("mikes:sick_localization");
      mikes_log(ML_ERR, "sick_localization error during waiting on new Data.");
      continue;
    }
    get_base_data(&base_data_local_copy);
    pthread_mutex_lock(&sick_localization_lock);
    process_all_data();
    pthread_mutex_unlock(&sick_localization_lock);
  }

  mikes_log(ML_INFO, "sick_localization quits.");
  threads_running_add(-1);
  return 0;
}

void tim_corner_new_data(corners_data *corners)
{
  if (pthread_mutex_trylock(&sick_localization_lock) == 0) {
    corners_local_copy = *corners;
    alert_new_data(fd);
    pthread_mutex_unlock(&sick_localization_lock);
  }
}

void init_sick_localization()
{
  if (!mikes_config.use_sick_localization)
  {
    mikes_log(ML_INFO, "sick_localization supressed by config.");
    online = 0;
    return;
  }
  online = 1;

  if (pipe(fd) != 0)
  {
    perror("mikes:sick_localization");
    mikes_log(ML_ERR, "creating pipe for sick localization");
    return;
  }

  pthread_t t;
  pthread_mutex_init(&sick_localization_lock, 0);
  register_tim_corner_callback(tim_corner_new_data);
  if (pthread_create(&t, 0, sick_localization_thread, 0) != 0)
  {
    perror("mikes:sick_localization");
    mikes_log(ML_ERR, "creating thread for sick localization");
  }
  else threads_running_add(1);
}

void shutdown_tim_hough_transform()
{
  close(fd[0]);
  close(fd[1]);
}

// ----------------------------------------------------------------
// ----------------------------------------------------------------
// --------------------------CALLBACK-----------------------------
// ----------------------------------------------------------------
// ----------------------------------------------------------------

void register_tim_hough_transform_callback(tim_hough_transform_receive_data_callback callback)
{
  if (!online) return;

  if (callbacks_count >= MAX_TIM_HOUGH_TRANSFORM_CALLBACKS)
  {
     mikes_log(ML_ERR, "too many tim_hough_transform callbacks");
     return;
  }
  callbacks[callbacks_count++] = callback;
}

void unregister_tim_hough_transform_callback(tim_hough_transform_receive_data_callback callback)
{
  if (!online) return;

  for (int i = 0; i < callbacks_count; i++) {
    if (callbacks[i] == callback) {
       callbacks[i] = callbacks[(callbacks_count--) - 1];

    }
  }
}
