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
  if ((segment1->start.x == segment2->start.x && segment1->start.y == segment2->start.y && segment1->end.x == segment2->end.x && segment1->end.y == segment2->end.y) ||
      (segment1->start.x == segment2->end.x && segment1->start.y == segment2->end.y && segment1->start.x == segment2->start.x && segment1->end.y == segment2->end.y)) {
    return 1;
  }
  return 0;
}

int pol_segment_comparator(const void *a, const void *b) {
  segment_data segmentA = (*(pol_segment_t*)a).segment;
  segment_data segmentB = (*(pol_segment_t*)b).segment;

  vector_2d vectorA;
  vector_from_two_points(&segmentA.start, &segmentA.end, &vectorA);

  vector_2d vectorB;
  vector_from_two_points(&segmentB.start, &segmentB.end, &vectorB);

  point_2d middleA = segmentA.start;
  middleA.x = middleA.x + vectorA.x / 2;

  point_2d middleB = segmentB.start;
  vectorB.x = middleB.x + vectorB.x / 2;

  vector_2d vectorMiddleA;
  vectorMiddleA.x = middleA.x;
  vectorMiddleA.y = middleA.y;

  vector_2d vectorMiddleB;
  vectorMiddleB.x = middleB.x;
  vectorMiddleB.y = middleB.y;

  double angleA = angle_from_axis_x(&vectorMiddleA);
  double angleB = angle_from_axis_x(&vectorMiddleB);

  return angleA - angleB;
}

int get_number_of_combinations_items_to_holes(int items, int holes, int start)
{
  if (items == 0) {
    return 1;
  }

  int total = 0;

  for (int index = start; index < holes; index++) {
    total = total + get_number_of_combinations_items_to_holes(items - 1, holes, index);
  }

  return total;
}

#define LINE_TO_SEGMENT_MULTIPLIER 10.0

double get_difference_between_segment_and_line(segment_data *segment, line *wall)
{
  point_2d start = {
    .x = wall.x1 * LINE_TO_SEGMENT_MULTIPLIER,
    .y = wall.y1 * LINE_TO_SEGMENT_MULTIPLIER
  };

  point_2d end = {
    .x = wall.x1 * LINE_TO_SEGMENT_MULTIPLIER,
    .y = wall.y1 * LINE_TO_SEGMENT_MULTIPLIER
  };

  vector_2d wall_vector;
  vector_from_two_points(&start, &end, &wall_vector);

  double wall_length = get_vector_length(&wall_vector);

  return segment->length - wall_length;
}

double get_difference_of_combination(pol_segments_t *found_segments, int *combinations, int length, int offset)
{
  double total_difference = 0;

  // found_segments->count == length - 1
  for (int index = 0, total_offset = offset; index < length - 1; index++, total_offset++) {
    total_offset += combinations[index];
    int line_index = total_offset % map_lines_count;

    pol_segment_t *found_segment = &found_segments->segments[index];
    line *wall = &map_lines[line_index];

    double difference += get_difference_between_segment_and_line(&found_segment->segment, wall);
    total_difference += fabs(difference);
  }

  return total_difference;
}

int get_pose_base_on_corners_and_heading(corners_data *corners, base_data_type *base_data, pose_type *result_pose)
{
  pol_segments_t found_segments;
  found_segments.count = 0;

  for (int c1_index = 0; c1_index < corners->count; c1_index++) {
    corner_data *corner1 = &corners->corners[c1_index];

    for (int c2_index = c1_index + 1; c2_index < corners->count; c2_index++) {
      corner_data *corner2 = &corners->corners[c2_index];

      if (are_equals_two_segments(&corner1->segment1, &corner2->segment1)) {
        found_segments.segments[found_segments.count].segment = corner1->segment1;
        found_segments.segments[found_segments.count].corner1 = *corner1;
        found_segments.segments[found_segments.count].corner2 = *corner2;
        found_segments.count = found_segments.count + 1;
      } else if (are_equals_two_segments(&corner1->segment2, &corner2->segment2)) {
        found_segments.segments[found_segments.count].segment = corner1->segment2;
        found_segments.segments[found_segments.count].corner1 = *corner1;
        found_segments.segments[found_segments.count].corner2 = *corner2;
        found_segments.count = found_segments.count + 1;
      } else if (are_equals_two_segments(&corner1->segment1, &corner2->segment2)) {
        found_segments.segments[found_segments.count].segment = corner1->segment1;
        found_segments.segments[found_segments.count].corner1 = *corner1;
        found_segments.segments[found_segments.count].corner2 = *corner2;
        found_segments.count = found_segments.count + 1;
      } else if (are_equals_two_segments(&corner1->segment2, &corner2->segment1)) {
        found_segments.segments[found_segments.count].segment = corner1->segment1;
        found_segments.segments[found_segments.count].corner1 = *corner1;
        found_segments.segments[found_segments.count].corner2 = *corner2;
        found_segments.count = found_segments.count + 1;
      }
    }
  }

  qsort(found_segments.segments, found_segments.count, sizeof(pol_segment_t), pol_segment_comparator);

  //for (int index_s = 0; index_s < found_segments.count; index_s++) {
    //printf("Sorted segment %10.4f %10.4f %10.4f %10.4f\n", found_segments.segments[index_s].segment.start.x, found_segments.segments[index_s].segment.start.y, found_segments.segments[index_s].segment.end.x, found_segments.segments[index_s].segment.end.y);
  //}

  int numberOfLines = map_lines_count; // our N
  int numberOfSegments = found_segments.count; // our M

  int numberOfItems = numberOfLines - 1;
  int numberOfHoles = numberOfSegments + 1;

  int numberOfCombinations = get_number_of_combinations_items_to_holes(numberOfItems, numberOfHoles, 0);

  int combinations[numberOfCombinations][numberOfHoles];

  // Initialize first combination
  for (int index = 0; index < numberOfHoles; index++) {
    combinations[0][index] = 0;
  }
  combinations[0][0] = numberOfItems;

  for (int index = 1; index < numberOfCombinations; index++) {

    // Create copy of combination before
    for (int copy_i = 0; copy_i < numberOfHoles; copy_i++) {
      combinations[index][copy_i] = combinations[index - 1][copy_i];
    }

    // Find first not zero value index before last index
    int lastNonEptyIndex = numberOfHoles - 2;
    while (lastNonEptyIndex > -1 && combinations[index][lastNonEptyIndex] == 0) {
      lastNonEptyIndex -= 1;
    }

    // Rotate numbers
    int lastValue = combinations[index][numberOfHoles - 1];

    combinations[index][lastNonEptyIndex] -= 1;
    combinations[index][numberOfHoles - 1] = 0;
    combinations[index][lastNonEptyIndex + 1] = 1 + lastValue;
  }

  int best_combination_i = -1;
  int best_start = -1;
  double best_combination_difference = INFINITY;

  for (int index = 0; index < numberOfCombinations; index++) {
    for (int start = 0; start < numberOfItems; start++) {
      double difference = get_difference_of_combination(&found_segments, combinations[index], numberOfHoles, start);
      if (best_combination_difference > difference) {
        best_combination_difference = difference;
        best_combination_i = index;
        best_start = start;
      }
    }
  }

  if (best_combination_i != -1) {
    printf("Best combination difference %10.4f offset %3d: ", best_combination_difference, best_start);
    for (int i = 0; i < numberOfHoles; i++) {
     printf("%3d ", combinations[best_combination_i][i]);
    }
    printf("\n");
  }

  // TODO get position
  // return POL_LOCALIZATION_SUCCESS;

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
  //print_sorted_map_lines();
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
