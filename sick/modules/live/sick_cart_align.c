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
#include "../../../mikes-common/modules/live/tim571.h"
#include "../../../mikes-common/modules/live/base_module.h"

#define MAX_SICK_CART_ALIGN_CALLBACKS 20

#define SCENE_DIFFERENCE_MAX_DISTANCE_TO_LOOK 4000

#define SCENE_DIFFERENCE_NOTICE_THRESHOLD 2000
#define SCENE_DIFFERENCE_STILL_THRESHOLD 350
#define FIRST_RAY_OF_INTEREST 290
#define LAST_RAY_OF_INTEREST 520

#define SATISFYING_ALIGNMENT_DISTANCE 100
#define VEHICLE_DISAPPEARED_TOO_FAR 550

#define FINAL_ALIGNMENT_TUNEUP_USEC 100000

#define ALIGNMENT_WAIT_TIMEOUT (60*1000000)

static pthread_mutex_t      sick_cart_align_lock;
static int                  fd[2];

sick_cart_align_t           result;

static sick_cart_align_receive_data_callback  callbacks[MAX_SICK_CART_ALIGN_CALLBACKS];
static int                                    callbacks_count;

static volatile int online;
static volatile int state;

static uint16_t             dist_local_copy[TIM571_DATA_COUNT];
static uint16_t             dist_remarkable_scene[TIM571_DATA_COUNT];

static base_data_type base_local_copy;

#define STATE_CART_ALIGN_IDLE 0
#define STATE_CART_ALIGN_WAITING 1
#define STATE_CART_ALIGN_WORKING 2

static volatile int data_loaded; 

void cart_align_laser_update(uint16_t *dist, uint8_t *rssi, tim571_status_data *status_data)
{
   if (state > STATE_CART_ALIGN_IDLE)
   {
     memcpy(dist_local_copy, dist, sizeof(uint16_t) * TIM571_DATA_COUNT);
     data_loaded = 1;
   }
}

void cart_align_base_update(base_data_type *data)
{
   if (state > STATE_CART_ALIGN_WAITING)
   {
     memcpy(&base_local_copy, data, sizeof(base_data_type));
   }
}


double compute_scene_difference()
{
    double scene_difference = 0.0;
    for (int i = FIRST_RAY_OF_INTEREST; i < LAST_RAY_OF_INTEREST; i++)
    {
      double diff = 0.0;
      if (dist_local_copy[i] < SCENE_DIFFERENCE_MAX_DISTANCE_TO_LOOK)
        diff = fabs(dist_local_copy[i] - dist_remarkable_scene[i]) / 10.0;
      scene_difference += diff;
    }
    return scene_difference;
}

#define RIGHTMOST  1
#define LEFTMOST   2
int find_last_free_ray(int side, int expected_distance_to_be_free)
{
  int i1, i2, di, last;

  if (side == RIGHTMOST) 
  {
     i1 = 135;
     i2 = 405 - 4;
     di = 1;
  }
  else 
  {
     i1 = 675 - 4;
     i2 = 405; 
     di = -1;
  }
  
  for (int i = i1; i != i2; i += di)
  {
    int free = 1;
    for (int j = 0; j < 4; j ++)
    {
      if (dist_remarkable_scene[i + j] < expected_distance_to_be_free)
      {
        free = 0;
        break;
      }
    }
    if (free) last = i + 2;
  }  
  return last;
}

int determine_distance_to_cart(int *nearest_ray)
{
  int min_dist_mm = 10000;
  *nearest_ray = FIRST_RAY_OF_INTEREST + LAST_RAY_OF_INTEREST / 2;

  for (int i = FIRST_RAY_OF_INTEREST; i < LAST_RAY_OF_INTEREST - 5; i++)
  {
    long d = 0;
    for (int j = 0; j < 5; j++)
      d += dist_remarkable_scene[i + j];
    if (d < min_dist_mm) 
    {
      min_dist_mm = d; 
      *nearest_ray = i + 2;
    }
  }
  return min_dist_mm / 5;
}


uint8_t process_align_cart()
{
  long long time_wait_for_alignment_started = usec();
  mikes_log(ML_INFO, "aligning to cart");
  data_loaded = 0;
  // waiting for the cart to arrive 
  // copy initial scene
  state = STATE_CART_ALIGN_WAITING;
  while (program_runs && !data_loaded) usleep(20000);
  if (!program_runs) return SICK_CART_ALIGN_FAIL;
  mikes_log(ML_INFO, "align data");

  memcpy(dist_remarkable_scene, dist_local_copy, sizeof(uint16_t) * TIM571_DATA_COUNT);

  //wait until scene changes significantly
  int scene_has_changed = 0;
  while (!scene_has_changed && program_runs)
  {
    usleep(20000);
    double scene_difference = compute_scene_difference();
    if (scene_difference > SCENE_DIFFERENCE_NOTICE_THRESHOLD) scene_has_changed = 1;
    //printf("scene diff: %.3lf\n", scene_difference);
    if (usec() - time_wait_for_alignment_started > ALIGNMENT_WAIT_TIMEOUT)
    {
       state = STATE_CART_ALIGN_IDLE;
       return SICK_CART_ALIGN_TIMEOUT;
    }
  }
  if (!program_runs) return SICK_CART_ALIGN_FAIL;
  mikes_log(ML_INFO, "cart arrives");
  say("Its coming");
  
  // wait until it stops changing (cart will stop)
  int scene_changing = 1;
  memcpy(dist_remarkable_scene, dist_local_copy, sizeof(uint16_t) * TIM571_DATA_COUNT);
  while (program_runs && scene_changing)
  {
    usleep(500000);
    double scene_difference = compute_scene_difference();
    if (scene_difference < SCENE_DIFFERENCE_STILL_THRESHOLD) scene_changing = 0;
    memcpy(dist_remarkable_scene, dist_local_copy, sizeof(uint16_t) * TIM571_DATA_COUNT);
    //printf("scene diff: %.3lf\n", scene_difference);
  }
  if (!program_runs) return SICK_CART_ALIGN_FAIL;
  mikes_log(ML_INFO, "cart has stopped");
  say("stopped");

  // now keep computing where the vehicle has landed and aligning
  int happy_with_alignment = 0;
  state = STATE_CART_ALIGN_WORKING;
  //int cnt = 0;
  while (program_runs && !happy_with_alignment)
  {
    int nearest_ray;
    int distance_to_vehicle = determine_distance_to_cart(&nearest_ray);

    int rightmost_free_ray = find_last_free_ray(RIGHTMOST, distance_to_vehicle + 300);
    int leftmost_free_ray = find_last_free_ray(LEFTMOST, distance_to_vehicle + 300);

    //printf("maxticks=%d", MM2COUNTER(distance_to_vehicle));
    set_max_ticks(MM2COUNTER(distance_to_vehicle));

/*
    cnt++;
    if (cnt % 5 == 0)
      printf("leftmost: %d (%.3lf deg), rightmost: %d (%.3lf deg), distance: %d\n", 
             leftmost_free_ray, tim571_ray2azimuth(leftmost_free_ray),
             rightmost_free_ray, tim571_ray2azimuth(rightmost_free_ray),
             distance_to_vehicle); 
 */

    if (distance_to_vehicle < SATISFYING_ALIGNMENT_DISTANCE) 
    {
      usleep(FINAL_ALIGNMENT_TUNEUP_USEC);
      happy_with_alignment = 1;
      stop_now();
    }
    else if (distance_to_vehicle > VEHICLE_DISAPPEARED_TOO_FAR)
    {
      stop_now();
      state = STATE_CART_ALIGN_IDLE;
      say("missed it");
      return SICK_CART_ALIGN_FAIL;
    }
    else
    {
      int cart_azimuth = (tim571_ray2azimuth(leftmost_free_ray) + tim571_ray2azimuth(rightmost_free_ray)) / 2;
      //printf("L=%d, R=%d\n", 12 + 12 * cart_azimuth / 60, 12 - 12 * cart_azimuth / 60);
      set_motor_speeds(12 + 12 * cart_azimuth / 40, 12 - 12 * cart_azimuth / 40);
      usleep(10000);
      memcpy(dist_remarkable_scene, dist_local_copy, sizeof(uint16_t) * TIM571_DATA_COUNT);
    }
  }    
  say("aligned");
  mikes_log(ML_INFO, "aligned");

  state = STATE_CART_ALIGN_IDLE;
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
    //pthread_mutex_lock(&sick_cart_align_lock);
    process_new_request();
    //pthread_mutex_unlock(&sick_cart_align_lock);
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
  mikes_log(ML_INFO, "align idle");
  state = STATE_CART_ALIGN_IDLE;

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

  register_tim571_callback(cart_align_laser_update);
  register_base_callback(cart_align_base_update);
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
