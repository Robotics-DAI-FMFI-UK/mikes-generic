#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "pol_localization.h"
#include "../../../mikes-common/bites/mikes.h"
#include "../../../mikes-common/bites/util.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"
#include "../../../mikes-common/modules/live/tim_hough_transform.h"
#include "../../../mikes-common/modules/live/tim_corner.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/passive/line_map.h"
#include "core/config_mikes.h"

#define MAX_POL_LOCALIZATION_CALLBACKS 20

static pthread_mutex_t      pol_localization_lock;
static int                  fd[2];

static corners_data         corners_local_copy;
static base_data_type       base_data_local_copy;

static int map_lines_count;
static line map_lines[MAX_LINES_IN_LINE_MAP];

static pol_localization_t   localization_data_local;

static pol_localization_receive_data_callback  callbacks[MAX_POL_LOCALIZATION_CALLBACKS];
static int                                     callbacks_count;

static int online;

int are_equals_two_segments(segment_data *segment1, segment_data *segment2)
{
  if ((segment1->start.x == segment2.start.x && segment1->start.y == segment2->start.y && segment1->end.x == segment2->end.x && segment1->end.y == segment2->end.y) ||
      (segment1->start.x == segment2.end.x && segment1->start.y == segment2->end.y && segment1->start.x == segment2->start.x && segment1->end.y == segment2->end.y)) {
    return 1;
  }
  return 0;
}

int get_pose_base_on_corners_and_heading(corners_data *corners, base_data_type *base_data, pose_type *result_pose)
{
  pol_segments_t segments;
  segments.count = 0;

  for (int c1_index = 0; c1_index < corners->count; c1_index++) {
    corner_data *corner1 = &corners->corners[c1_index];

    for (int c2_index = c1_index + 1; c2_index < corners->count; c2_index++) {
      corner_data *corner2 = &corners->corners[c2_index];

      if (are_equals_two_segments(&corner1->segment1, &corner2->segment1)) {
        // corner1->segment1 == corner2->segment1
        segments.segments[segments.count].segment = corner1->segment1;
        segments.segments[segments.count].corner1 = *corner1;
        segments.segments[segments.count].corner2 = *corner2;
        segments.count = segments.count + 1;
      } else if (are_equals_two_segments(&corner1->segment2, &corner2->segment2)) {
        // corner1->segment2 == corner2->segment2
        segments.segments[segments.count].segment = corner1->segment2;
        segments.segments[segments.count].corner1 = *corner1;
        segments.segments[segments.count].corner2 = *corner2;
        segments.count = segments.count + 1;
      } else if (are_equals_two_segments(&corner1->segment1, &corner2->segment2)) {
        // corner1->segment1 == corner2->segment2
        segments.segments[segments.count].segment = corner1->segment1;
        segments.segments[segments.count].corner1 = *corner1;
        segments.segments[segments.count].corner2 = *corner2;
        segments.count = segments.count + 1;
      } else if (are_equals_two_segments(&corner1->segment2, &corner2->segment1)) {
        // corner1->segment2 == corner2->segment1
        segments.segments[segments.count].segment = corner1->segment1;
        segments.segments[segments.count].corner1 = *corner1;
        segments.segments[segments.count].corner2 = *corner2;
        segments.count = segments.count + 1;
      }
    }
  }
  // TODO
  // return POL_LOCALIZATION_SUCCESS;

  printf("Number of segments found %d \n", segments.count);

  return POL_LOCALIZATION_FAIL;
}

void process_all_data()
{
  localization_data_local.status = get_pose_base_on_corners_and_heading(&corners_local_copy, &base_data_local_copy, &localization_data_local.pose);
  for (int i = 0; i < callbacks_count; i++) {
    callbacks[i](&localization_data_local);
  }
}

void *pol_localization_thread(void *args)
{
  while (program_runs)
  {
    if (wait_for_new_data(fd) < 0) {
      perror("mikes:pol_localization");
      mikes_log(ML_ERR, "pol_localization error during waiting on new Data.");
      continue;
    }
    get_base_data(&base_data_local_copy);
    pthread_mutex_lock(&pol_localization_lock);
    process_all_data();
    pthread_mutex_unlock(&pol_localization_lock);
  }

  mikes_log(ML_INFO, "pol_localization quits.");
  threads_running_add(-1);
  return 0;
}

void tim_corner_new_data(corners_data *corners)
{
  if (pthread_mutex_trylock(&pol_localization_lock) == 0)
  {
    corners_local_copy = *corners;
    alert_new_data(fd);
    pthread_mutex_unlock(&pol_localization_lock);
  }
}

void sort_map_lines_as_polygon()
{
  if (map_lines_count < 1) return;

  line sorted_lines[map_lines_count];
  int lines_used[map_lines_count];
  memset(lines_used, 0, sizeof(int) * map_lines_count);
  int sorted = 0;

  sorted_lines[0] = map_lines[0];
  lines_used[0] = 1;
  sorted++;

  while (sorted < map_lines_count) {
    line last_line = sorted_lines[sorted - 1];
    int success = 0;

    for (int line_i = 0; line_i < map_lines_count; line_i++) {
      if (!lines_used[line_i]) {
        line courent_line = map_lines[line_i];

        if (last_line.x2 == courent_line.x1 && last_line.y2 == courent_line.y1) {
          sorted_lines[sorted].x1 = courent_line.x1;
          sorted_lines[sorted].y1 = courent_line.y1;
          sorted_lines[sorted].x2 = courent_line.x2;
          sorted_lines[sorted].y2 = courent_line.y2;
          sorted_lines[sorted].id = courent_line.id;
          lines_used[line_i] = 1;
          sorted++;
          success = 1;
          break;
        } else if (last_line.x2 == courent_line.x2 && last_line.y2 == courent_line.y2) {
          sorted_lines[sorted].x1 = courent_line.x2;
          sorted_lines[sorted].y1 = courent_line.y2;
          sorted_lines[sorted].x2 = courent_line.x1;
          sorted_lines[sorted].y2 = courent_line.y1;
          sorted_lines[sorted].id = courent_line.id;
          lines_used[line_i] = 1;
          sorted++;
          success = 1;
          break;
        }
      }
    }

    if (!success) {
      printf("Unexpected failure pol_localization sort\n");
      return;
    }
  }

  memcpy(map_lines, sorted_lines, sizeof(line) * map_lines_count);
}

void print_sorted_map_lines()
{
  for (int index_l = 0; index_l < map_lines_count; index_l++) {
    printf("Line X1: %10.4f Y1: %10.4f X2: %10.4f Y2: %10.4f\n", map_lines[index_l].x1, map_lines[index_l].y1, map_lines[index_l].x2, map_lines[index_l].y2);
  }
}

void init_pol_localization()
{
  if (!mikes_config.use_pol_localization)
  {
    mikes_log(ML_INFO, "pol_localization supressed by config.");
    online = 0;
    return;
  }
  online = 1;

  // tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_SINGLE);

  if (pipe(fd) != 0)
  {
    perror("mikes:pol_localization");
    mikes_log(ML_ERR, "creating pipe for pol localization");
    return;
  }

  get_line_map_data(map_lines, &map_lines_count);
  sort_map_lines_as_polygon();

  pthread_t t;
  pthread_mutex_init(&pol_localization_lock, 0);
  register_tim_corner_callback(tim_corner_new_data);
  if (pthread_create(&t, 0, pol_localization_thread, 0) != 0)
  {
    perror("mikes:pol_localization");
    mikes_log(ML_ERR, "creating thread for pol localization");
  }
  else threads_running_add(1);
}

void shutdown_pol_localization()
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

void register_pol_localization_callback(pol_localization_receive_data_callback callback)
{
  if (!online) return;

  if (callbacks_count >= MAX_POL_LOCALIZATION_CALLBACKS)
  {
     mikes_log(ML_ERR, "too many pol_localization callbacks");
     return;
  }
  callbacks[callbacks_count++] = callback;
}

void unregister_pol_localization_callback(pol_localization_receive_data_callback callback)
{
  if (!online) return;

  for (int i = 0; i < callbacks_count; i++) {
    if (callbacks[i] == callback) {
       callbacks[i] = callbacks[(callbacks_count--) - 1];
    }
  }
}
