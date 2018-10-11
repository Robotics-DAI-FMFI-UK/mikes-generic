#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "sick_localization.h"
#include "../../../mikes-common/bites/mikes.h"
#include "../../../mikes-common/bites/util.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "../../../mikes-common/modules/live/tim_hough_transform.h"
#include "../../../mikes-common/modules/live/tim_corner.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "core/config_mikes.h"

#define MAX_SICK_LOCALIZATION_CALLBACKS 20

#define RIGHT_TOP_CORNER 0
#define RIGHT_BOTTOM_CORNER 1
#define LEFT_BOTTOM_CORNER 2
#define LEFT_TOP_CORNER 3

static pthread_mutex_t      sick_localization_lock;
static int                  fd[2];

static corners_data         corners_local_copy;
static base_data_type       base_data_local_copy;

static pose_type            pose_localization_local;

static sick_localization_receive_data_callback  callbacks[MAX_SICK_LOCALIZATION_CALLBACKS];
static int                                      callbacks_count;

static int online;

static point_2d entry = {
  .x = 0,
  .y = 0
};

static point_2d right_top_corner = {
  .x = SICK_MAP_WITH_IN_MM,
  .y = SICK_MAP_HEIGHT_IN_MM
};

static point_2d right_bottom_corner = {
  .x = SICK_MAP_WITH_IN_MM,
  .y = 0
};

static point_2d left_bottom_corner = {
  .x = 0,
  .y = 0
};

static point_2d left_top_corner = {
  .x = 0,
  .y = SICK_MAP_HEIGHT_IN_MM
};

// ----------------------------------------------------------------
// ----------------------------------------------------------------
// --------------------------LIFECYCLE-----------------------------
// ----------------------------------------------------------------
// ----------------------------------------------------------------

int get_corner_index(corner_data *corner, double heading)
{
  vector_2d corner_vector;
  vector_from_two_points(&entry, &corner->corner, &corner_vector);
  double tim_angle = angle_from_axis_x(&corner_vector);
  double map_angle = tim571_angle_and_compass_heading_to_map_angle(tim_angle, heading);
  if (map_angle < 90 ) return RIGHT_TOP_CORNER;
  if (map_angle < 180) return RIGHT_BOTTOM_CORNER;
  if (map_angle < 270) return LEFT_BOTTOM_CORNER;
  return LEFT_TOP_CORNER;
}

void get_left_and_right_line_from_corner(corner_data *corner, angle_line_2d *line_left, angle_line_2d *line_right)
{
  if (angle_difference(corner->segment1.line.angle, corner->segment2.line.angle) < 0.0) {
    *line_left = corner->segment1.line;
    *line_right = corner->segment2.line;
  } else {
    *line_left = corner->segment2.line;
    *line_right = corner->segment1.line;
  }
}

void get_difference_x_and_y(int *corner, angle_line_2d *line_left, angle_line_2d *line_right, double *difference_x, double *difference_y)
{
  switch (*corner) {
    case RIGHT_TOP_CORNER:
    case LEFT_BOTTOM_CORNER:
      *difference_x = line_right->distance;
      *difference_y = line_left->distance;
      break;
    case RIGHT_BOTTOM_CORNER:
    case LEFT_TOP_CORNER:
      *difference_x = line_left->distance;
      *difference_y = line_right->distance;
      break;
    default:
      printf("UNKNOWN CORNER %d\n", *corner);
      return;
  }
}

void get_map_x_and_y(int *corner, double *difference_x, double *difference_y, double *x, double *y)
{
  switch (*corner) {
    case RIGHT_TOP_CORNER:
      *x = SICK_MAP_WITH_IN_MM - *difference_x;
      *y = SICK_MAP_HEIGHT_IN_MM - *difference_y;
      break;
    case RIGHT_BOTTOM_CORNER:
      *x = SICK_MAP_WITH_IN_MM - *difference_x;
      *y = *difference_y;
      break;
    case LEFT_BOTTOM_CORNER:
      *x = *difference_x;
      *y = *difference_y;
      break;
    case LEFT_TOP_CORNER:
      *x = *difference_x;
      *y = SICK_MAP_HEIGHT_IN_MM - *difference_y;
      break;
    default:
      printf("UNKNOWN CORNER %d\n", *corner);
      return;
  }
}

void get_heading(corner_data *corner, int *corner_index, double *x, double *y, double* heading)
{
  point_2d robot_p;
  robot_p.x = *x;
  robot_p.y = *y;

  vector_2d corner_v;
  switch (*corner_index) {
    case RIGHT_TOP_CORNER:
      vector_from_two_points(&robot_p, &right_top_corner, &corner_v);
      break;
    case RIGHT_BOTTOM_CORNER:
      vector_from_two_points(&robot_p, &right_bottom_corner, &corner_v);
      break;
    case LEFT_BOTTOM_CORNER:
      vector_from_two_points(&robot_p, &left_bottom_corner, &corner_v);
      break;
    case LEFT_TOP_CORNER:
      vector_from_two_points(&robot_p, &left_top_corner, &corner_v);
      break;
    default:
      printf("UNKNOWN CORNER %d\n", *corner_index);
      return;
  }
  double map_corner_angle = math_azimuth_to_robot_azimuth(angle_from_axis_x(&corner_v));

  vector_2d corner_tim_v;
  vector_from_two_points(&entry, &corner->corner, &corner_tim_v);
  double robot_angle_to_corner = math_azimuth_to_robot_azimuth(angle_from_axis_x(&corner_tim_v));

  *heading = normAlpha(map_corner_angle - robot_angle_to_corner) * RADIAN;
}

int get_pose_base_on_corners_and_heading(corners_data *corners, base_data_type *base_data, pose_type *result_pose)
{
  for (int c_index = 0; c_index < corners->count; c_index++) {
    corner_data *corner = &corners->corners[c_index];
    angle_line_2d line_left, line_right;
    double difference_x, difference_y;

    int corner_index = get_corner_index(corner, base_data->heading);
    get_left_and_right_line_from_corner(corner, &line_left, &line_right);
    get_difference_x_and_y(&corner_index, &line_left, &line_right, &difference_x, &difference_y);
    get_map_x_and_y(&corner_index, &difference_x, &difference_y, &result_pose->x, &result_pose->y);
    get_heading(corner, &corner_index, &result_pose->x, &result_pose->y, &result_pose->heading);
    if (result_pose->x > 0 && result_pose->x < SICK_MAP_WITH_IN_MM && result_pose->y > 0  && result_pose->y < SICK_MAP_HEIGHT_IN_MM) {
      return 1;
    }
  }
  return 0;
}

void process_all_data()
{
  if (get_pose_base_on_corners_and_heading(&corners_local_copy, &base_data_local_copy, &pose_localization_local)) {
    for (int i = 0; i < callbacks_count; i++) {
      callbacks[i](&pose_localization_local);
    }
  }
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
  if (pthread_mutex_trylock(&sick_localization_lock) == 0)
  {
    corners_local_copy = *corners;
    alert_new_data(fd);
    pthread_mutex_unlock(&sick_localization_lock);
  }
}

void request_actualize_pose(int cmd)
{
  switch (cmd) {
    case NAVIG_START_LOCALIZE:
      tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_CONTINUOUS);
      break;
    case NAVIG_STOP_LOCALIZE:
      tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_SINGLE);
      break
    default:
      perror("mikes:sick_localization unknown cmd %d\n", cmd);
      break;
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

  tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_SINGLE);
  navig_register_actualize_pose_function(request_actualize_pose);

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

void shutdown_sick_localization()
{
  online = 0;

  close(fd[0]);
  close(fd[1]);
}

// ----------------------------------------------------------------
// ----------------------------------------------------------------
// --------------------------CALLBACK-----------------------------
// ----------------------------------------------------------------
// ----------------------------------------------------------------

void register_sick_localization_callback(sick_localization_receive_data_callback callback)
{
  if (!online) return;

  if (callbacks_count >= MAX_SICK_LOCALIZATION_CALLBACKS)
  {
     mikes_log(ML_ERR, "too many sick_localization callbacks");
     return;
  }
  callbacks[callbacks_count++] = callback;
}

void unregister_sick_localization_callback(sick_localization_receive_data_callback callback)
{
  if (!online) return;

  for (int i = 0; i < callbacks_count; i++) {
    if (callbacks[i] == callback) {
       callbacks[i] = callbacks[(callbacks_count--) - 1];
    }
  }
}
