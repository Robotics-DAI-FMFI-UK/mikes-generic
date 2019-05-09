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

#define LINE_TO_SEGMENT_MULTIPLIER 10.0
#define ANGLE_MAXIMUM 300
#define MAX_ACCEPTABLE_DIFFERENCE 150

static pthread_mutex_t      pol_localization_lock;
static int                  fd[2];

static corners_data         corners_local_copy;
static base_data_type       base_data_local_copy;

static int map_lines_count;
static line map_lines[MAX_LINES_IN_LINE_MAP];

static int number_of_verticles;
static point_2d map_verticles[MAX_LINES_IN_LINE_MAP + 1];

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

int are_equals_two_corners(point_2d *corner1, point_2d *corner2)
{
  if (corner1->x == corner2->x && corner1->y == corner2->y) {
    return 1;
  }

  return 0;
}

int have_common_corner(pol_segment_t *segment1, pol_segment_t *segment2)
{
  if (are_equals_two_corners(&segment1->corner1.corner, &segment2->corner1.corner) ||
      are_equals_two_corners(&segment1->corner2.corner, &segment2->corner2.corner) ||
      are_equals_two_corners(&segment1->corner1.corner, &segment2->corner2.corner) ||
      are_equals_two_corners(&segment1->corner2.corner, &segment2->corner1.corner)) {
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

  return angleB - angleA;
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

double get_difference_between_segment_and_line(segment_data *segment, line *wall)
{
  point_2d start = {
    .x = wall->x1 * LINE_TO_SEGMENT_MULTIPLIER,
    .y = wall->y1 * LINE_TO_SEGMENT_MULTIPLIER
  };

  point_2d end = {
    .x = wall->x2 * LINE_TO_SEGMENT_MULTIPLIER,
    .y = wall->y2 * LINE_TO_SEGMENT_MULTIPLIER
  };

  vector_2d wall_vector;
  vector_from_two_points(&start, &end, &wall_vector);

  double wall_length = get_vector_length(&wall_vector);

  return segment->length - wall_length;
}

// found_segments -> all found segments in sequence
// combines -> how many segments must be in sequence for current index
// combinations -> how many spaces must be between every combines for current index
void get_position_representation_from_combination(int *combines, int *combinations, int numberOfCombinedSegments, int offset, int *positions)
{
  for (int index = 0, segment_index = 0, total_offset = offset; index < numberOfCombinedSegments; index++) {
    total_offset += combinations[index];

    for (int index_combined = 0; index_combined < combines[index]; index_combined++, segment_index++) {
      int line_index = (total_offset + segment_index) % map_lines_count;
      positions[segment_index] = line_index;
    }
  }
}

double get_difference_of_combination(pol_segments_t *found_segments, int *combines, int *combinations, int numberOfCombinedSegments, int offset)
{
  double total_difference = 0;

  int positions[found_segments->count];
  get_position_representation_from_combination(combines, combinations, numberOfCombinedSegments, offset, positions);

  for (int index = 0; index < found_segments->count; index++) {
    int line_index = positions[index];
    line *wall = &map_lines[line_index];

    pol_segment_t *found_segment = &found_segments->segments[index];

    double difference = get_difference_between_segment_and_line(&found_segment->segment, wall);
    total_difference += fabs(difference);
  }

  return total_difference;
}

int is_in_polygon(point_2d *verticles, int length, point_2d *test)
{
  int i, j, c = 0;
  for (i = 0, j = length - 1; i < length; j = i++) {
    if ( ((verticles[i].y > test->y) != (verticles[j].y > test->y)) &&
          (test->x < (verticles[j].x - verticles[i].x) * (test->y - verticles[i].y) / (verticles[j].y - verticles[i].y) + verticles[i].x) ) {
      c = !c;
    }
  }

  return c;
}

// ----------------------------------------------------------
// TODO Move to math_2d
// ----------------------------------------------------------

typedef struct point_circle_2d {
  double x;
  double y;
  double r;
} circle_2d;

int two_circles_intersection(circle_2d *c1, circle_2d *c2, point_2d *result1, point_2d *result2) {
  double val1, val2, test;
  double D = sqrt((c1->x - c2->x) * (c1->x - c2->x) + (c1->y - c2->y) * (c1->y - c2->y));

  if (((c1->r + c2->r) >= D) && (D >= fabs(c1->r - c2->r))) {
    double a1 = D + c1->r + c2->r;
    double a2 = D + c1->r - c2->r;
    double a3 = D - c1->r + c2->r;
    double a4 = -D + c1->r + c2->r;
    double area = sqrt(a1 * a2 * a3 * a4) / 4;

    val1 = (c1->x + c2->x) / 2 + (c2->x - c1->x) * (c1->r * c1->r - c2->r * c2->r) / (2 * D * D);
    val2 = 2 * (c1->y - c2->y) * area / (D * D);
    result1->x = val1 + val2;
    result2->x = val1 - val2;

    val1 = (c1->y + c2->y) / 2 + (c2->y - c1->y) * (c1->r * c1->r - c2->r * c2->r) / (2 * D * D);
    val2 = 2 * (c1->x - c2->x) * area / (D * D);
    result1->y = val1 - val2;
    result2->y = val1 + val2;

    test = fabs((result1->x - c1->x) * (result1->x - c1->x) + (result1->y - c1->y) * (result1->y - c1->y) - c1->r * c1->r);
    if (test > 0.0000001) {
      double tmp = result1->y;
      result1->y = result2->y;
      result2->y = tmp;
    }

    return 1;
  }

  return 0;
}

// ----------------------------------------------------------
// End
// ----------------------------------------------------------

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

  if (found_segments.count < 1 || found_segments.count > map_lines_count) {
    printf("FAIL NOT ENOUGH SEGMENTS\n");
    return POL_LOCALIZATION_FAIL;
  }

  qsort(found_segments.segments, found_segments.count, sizeof(pol_segment_t), pol_segment_comparator);

  int combined_segments[found_segments.count];
  int combined_segments_length = 1;
  combined_segments[combined_segments_length - 1] = 1;

  for (int index = 1; index < found_segments.count; index++) {
    pol_segment_t *segment1 = &found_segments.segments[index - 1];
    pol_segment_t *segment2 = &found_segments.segments[index];

    if (have_common_corner(segment1, segment2)) {
      combined_segments[combined_segments_length - 1]++;
    } else {
      combined_segments[combined_segments_length] = 1;
      combined_segments_length++;
    }
  }

  // printf("Combined segments result: ");
  // for (int index = 0; index < combined_segments_length; index++) {
  //   printf("%3d ", combined_segments[index]);
  // }
  // printf("\n");

  // for (int index_s = 0; index_s < found_segments.count; index_s++) {
  //   printf("Sorted segment %10.4f %10.4f %10.4f %10.4f\n", found_segments.segments[index_s].segment.start.x, found_segments.segments[index_s].segment.start.y, found_segments.segments[index_s].segment.end.x, found_segments.segments[index_s].segment.end.y);
  // }

  int numberOfLines = map_lines_count; // our N
  int numberOfCombinedSegments = combined_segments_length; // our M

  int numberOfItems = numberOfLines - 1;
  int numberOfHoles = numberOfCombinedSegments + 1;

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
      double difference = get_difference_of_combination(&found_segments, combined_segments, combinations[index], numberOfCombinedSegments, start);
      // printf("Combination difference %10.4f offset %3d: ", difference, start);
      // for (int i = 0; i < numberOfHoles; i++) {
      //  printf("%3d ", combinations[index][i]);
      // }
      // printf("\n");
      // sleep(2);
      if (best_combination_difference > difference) {
        best_combination_difference = difference;
        best_combination_i = index;
        best_start = start;
      }
    }
  }

  if (best_combination_i == -1) {
    printf("FAIL NOT BEST COMBINATION\n");
    return POL_LOCALIZATION_FAIL;
  }

  // printf("Segments %3d, Combined to %3d, Best combination difference %10.4f offset %3d: ",
  //       found_segments.count, combined_segments_length, best_combination_difference, best_start);
  // for (int i = 0; i < numberOfHoles; i++) {
  //   printf("%3d ", combinations[best_combination_i][i]);
  // }
  // printf("\n");

  point_2d entry = {
    .x = 0,
    .y = 0
  };

  int positions[found_segments.count];
  get_position_representation_from_combination(combined_segments, combinations[best_combination_i], numberOfCombinedSegments, best_start, positions);

  point_2d potencial_locations_1[found_segments.count];
  point_2d potencial_locations_1_real[found_segments.count];
  point_2d potencial_locations_1_sensor[found_segments.count];
  point_2d potencial_locations_2[found_segments.count];
  point_2d potencial_locations_2_real[found_segments.count];
  point_2d potencial_locations_2_sensor[found_segments.count];
  int potencial_length = 0;

  for (int index = 0; index < found_segments.count; index++) {
    int line_index = positions[index];

    line *wall = &map_lines[line_index];
    pol_segment_t *found_segment = &found_segments.segments[index];

    point_2d p_corner_segment_1 = found_segment->corner1.corner;
    point_2d p_corner_segment_2 = found_segment->corner2.corner;

    vector_2d v_corner_segment_1;
    vector_2d v_corner_segment_2;

    vector_from_two_points(&entry, &p_corner_segment_1, &v_corner_segment_1);
    vector_from_two_points(&entry, &p_corner_segment_2, &v_corner_segment_2);

    double angle1 = angle_from_axis_x(&v_corner_segment_1);
    double angle2 = angle_from_axis_x(&v_corner_segment_2);

    double segment_left_distance;
    double segment_right_distance;

    if ((angle1 < ANGLE_MAXIMUM && angle2 < ANGLE_MAXIMUM) || (angle1 >= ANGLE_MAXIMUM && angle2 >= ANGLE_MAXIMUM)) {
      if (angle1 > angle2) {
        segment_left_distance = get_vector_length(&v_corner_segment_1);
        potencial_locations_1_real[potencial_length].x = wall->x1;
        potencial_locations_1_real[potencial_length].y = wall->y1;
        potencial_locations_1_sensor[potencial_length] = p_corner_segment_1;
        segment_right_distance = get_vector_length(&v_corner_segment_2);
        potencial_locations_2_real[potencial_length].x = wall->x2;
        potencial_locations_2_real[potencial_length].y = wall->y2;
        potencial_locations_2_sensor[potencial_length] = p_corner_segment_2;
      } else {
        segment_left_distance = get_vector_length(&v_corner_segment_2);
        potencial_locations_1_real[potencial_length].x = wall->x2;
        potencial_locations_1_real[potencial_length].y = wall->y2;
        potencial_locations_1_sensor[potencial_length] = p_corner_segment_2;
        segment_right_distance = get_vector_length(&v_corner_segment_1);
        potencial_locations_2_real[potencial_length].x = wall->x1;
        potencial_locations_2_real[potencial_length].y = wall->y1;
        potencial_locations_2_sensor[potencial_length] = p_corner_segment_1;
      }
    } else {
      if (angle1 < ANGLE_MAXIMUM) {
        segment_left_distance = get_vector_length(&v_corner_segment_1);
        potencial_locations_1_real[potencial_length].x = wall->x1;
        potencial_locations_1_real[potencial_length].y = wall->y1;
        potencial_locations_1_sensor[potencial_length] = p_corner_segment_1;
        segment_right_distance = get_vector_length(&v_corner_segment_2);
        potencial_locations_2_real[potencial_length].x = wall->x2;
        potencial_locations_2_real[potencial_length].y = wall->y2;
        potencial_locations_2_sensor[potencial_length] = p_corner_segment_2;
      } else {
        segment_left_distance = get_vector_length(&v_corner_segment_2);
        potencial_locations_1_real[potencial_length].x = wall->x2;
        potencial_locations_1_real[potencial_length].y = wall->y2;
        potencial_locations_1_sensor[potencial_length] = p_corner_segment_2;
        segment_right_distance = get_vector_length(&v_corner_segment_1);
        potencial_locations_2_real[potencial_length].x = wall->x1;
        potencial_locations_2_real[potencial_length].y = wall->y1;
        potencial_locations_2_sensor[potencial_length] = p_corner_segment_1;
      }
    }

    circle_2d c1 = {
      .x = wall->x1,
      .y = wall->y1,
      .r = segment_left_distance / LINE_TO_SEGMENT_MULTIPLIER
    };

    circle_2d c2 = {
      .x = wall->x2,
      .y = wall->y2,
      .r = segment_right_distance / LINE_TO_SEGMENT_MULTIPLIER
    };


    if (two_circles_intersection(&c1, &c2, &potencial_locations_1[potencial_length], &potencial_locations_2[potencial_length])) {
      potencial_length++;
    }

    // printf("Input X1: %6.4f Y1: %6.4f R1 %6.4f X2: %6.4f Y2: %6.4f R2 %6.4f Result X1: %6.4f Y1: %6.4f X2: %6.4f Y2: %6.4f\n",
    //   c1.x, c1.y, c1.r, c2.x, c2.y, c2.r,
    //   potencial_locations_1[potencial_length - 1].x, potencial_locations_1[potencial_length - 1].y, potencial_locations_2[potencial_length - 1].x, potencial_locations_2[potencial_length - 1].y);
  }

  point_2d single_points_in_polygon[found_segments.count];
  point_2d single_real_corners[found_segments.count];
  point_2d single_sensor_corners[found_segments.count];
  int single_points_length = 0;

  point_2d double_points_in_polygon[found_segments.count * 2];
  point_2d double_real_corners[found_segments.count * 2];
  point_2d double_sensor_corners[found_segments.count * 2];
  int double_points_length = 0;

  for (int index = 0; index < potencial_length; index++) {
    int first_in_polygon = is_in_polygon(map_verticles, number_of_verticles, &potencial_locations_1[index]);
    int second_in_polygon = is_in_polygon(map_verticles, number_of_verticles, &potencial_locations_1[index]);

    if (first_in_polygon && !second_in_polygon) {
      single_points_in_polygon[single_points_length] = potencial_locations_1[index];
      single_real_corners[single_points_length] = potencial_locations_1_real[index];
      single_sensor_corners[single_points_length] = potencial_locations_1_sensor[index];
      single_points_length++;
    } else if (second_in_polygon && !first_in_polygon) {
      single_points_in_polygon[single_points_length] = potencial_locations_2[index];
      single_real_corners[single_points_length] = potencial_locations_2_real[index];
      single_sensor_corners[single_points_length] = potencial_locations_2_sensor[index];
      single_points_length++;
    } else if (first_in_polygon && second_in_polygon) {
      double_points_in_polygon[double_points_length] = potencial_locations_1[index];
      double_real_corners[double_points_length] = potencial_locations_1_real[index];
      double_sensor_corners[double_points_length] = potencial_locations_1_sensor[index];
      double_points_length++;
      double_points_in_polygon[double_points_length] = potencial_locations_2[index];
      double_real_corners[double_points_length] = potencial_locations_2_real[index];
      double_sensor_corners[double_points_length] = potencial_locations_2_sensor[index];
      double_points_length++;
    }
  }

  if (!single_points_length) {
    printf("FAIL NOT SINGLE POINT\n");
    return POL_LOCALIZATION_FAIL;
  }

  for (int index_1 = 0; index_1 < single_points_length; index_1++) {
    for (int index_2 = 1; index_2 < single_points_length; index_2++) {
      vector_2d difference_vector;
      vector_from_two_points(&single_points_in_polygon[index_1], &single_points_in_polygon[index_2], &difference_vector);

      double difference_vector_length = get_vector_length(&difference_vector);

      if (difference_vector_length > MAX_ACCEPTABLE_DIFFERENCE) {
        printf("This is pretty bad boys\n");
        return POL_LOCALIZATION_FAIL;
      }
    }
  }

  point_2d center_single_point = {
    .x = 0,
    .y = 0
  };

  for (int index = 0; index < single_points_length; index++) {
    center_single_point.x += single_points_in_polygon[index].x;
    center_single_point.y += single_points_in_polygon[index].y;
  }

  point_2d center_final_point = center_single_point;

  center_single_point.x = center_single_point.x / single_points_length;
  center_single_point.y = center_single_point.y / single_points_length;

  int double_points_used = 0;
  for (int index = 0; index < double_points_length;) {
    vector_2d difference_vector_1;
    vector_from_two_points(&center_single_point, &double_points_in_polygon[index], &difference_vector_1);
    double difference_vector_1_length = get_vector_length(&difference_vector_1);

    vector_2d difference_vector_2;
    vector_from_two_points(&center_single_point, &double_points_in_polygon[index + 1], &difference_vector_2);
    double difference_vector_2_length = get_vector_length(&difference_vector_2);

    if (difference_vector_1_length < difference_vector_2_length) {
      center_final_point.x += double_points_in_polygon[index].x;
      center_final_point.y += double_points_in_polygon[index].y;
    } else {
      center_final_point.x += double_points_in_polygon[index + 1].x;
      center_final_point.y += double_points_in_polygon[index + 1].y;
    }

    double_points_used++;
    index += 2;
  }

  center_final_point.x = center_final_point.x / (single_points_length + double_points_used);
  center_final_point.y = center_final_point.y / (single_points_length + double_points_used);

  sleep(5);
  printf("Found final localization X: %6.4f Y: %6.4f\n", center_final_point.x, center_final_point.y);

  for (int index = 0; index < single_points_length; index++) {
    vector_2d corner_v;
    vector_from_two_points(&center_final_point, &single_real_corners[index], &corner_v);
    double map_corner_angle = math_azimuth_to_robot_azimuth(angle_from_axis_x(&corner_v));

    vector_2d corner_tim_v;
    vector_from_two_points(&entry, &single_sensor_corners[index], &corner_tim_v);
    double robot_angle_to_corner = math_azimuth_to_robot_azimuth(angle_from_axis_x(&corner_tim_v));

    double alpha = normAlpha(map_corner_angle - robot_angle_to_corner);
    printf("Found potencional heading %4.4f\n", alpha);
  }

  return POL_LOCALIZATION_SUCCESS;
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

void print_sorted_map_lines()
{
  for (int index_l = 0; index_l < map_lines_count; index_l++) {
    printf("Line X1: %10.4f Y1: %10.4f X2: %10.4f Y2: %10.4f\n", map_lines[index_l].x1, map_lines[index_l].y1, map_lines[index_l].x2, map_lines[index_l].y2);
  }
}

void rotate_line_points_around_x_axis()
{
  for (int index = 0; index < map_lines_count; index++) {
    line *wall = &map_lines[index];
    wall->y1 = -wall->y1;
    wall->y2 = -wall->y2;
  }
}

void print_sorted_verticles()
{
  printf("Verticles:\n");
  for (int index = 0; index < number_of_verticles; index++) {
    printf("Point X: %10.4f Y: %10.4f\n", map_verticles[index].x, map_verticles[index].y);
  }
}

void sort_map_lines_as_polygon()
{
  if (map_lines_count < 1) return;

  line sorted_lines[map_lines_count];
  int lines_used[map_lines_count];
  memset(lines_used, 0, sizeof(int) * map_lines_count);
  int sorted = 0;

  sorted_lines[sorted] = map_lines[sorted];
  lines_used[sorted] = 1;
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
          success = 1;
        } else if (last_line.x2 == courent_line.x2 && last_line.y2 == courent_line.y2) {
          sorted_lines[sorted].x1 = courent_line.x2;
          sorted_lines[sorted].y1 = courent_line.y2;
          sorted_lines[sorted].x2 = courent_line.x1;
          sorted_lines[sorted].y2 = courent_line.y1;
          sorted_lines[sorted].id = courent_line.id;
          success = 1;
        }

        if (success) {
          lines_used[line_i] = 1;
          sorted++;
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
  // rotate_line_points_around_x_axis();


  // Collect sorted verticles for polygon
  map_verticles[0].x = map_lines[0].x1;
  map_verticles[0].y = map_lines[0].y1;
  number_of_verticles = 1;

  while (number_of_verticles - 1 < map_lines_count) {
    map_verticles[number_of_verticles].x = map_lines[number_of_verticles - 1].x2;
    map_verticles[number_of_verticles].y = map_lines[number_of_verticles - 1].y2;
    number_of_verticles++;
  }

  // print_sorted_map_lines();
  // print_sorted_verticles();
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
