#include <pthread.h>
#include <unistd.h>
#include <stdio.h>

#include "core/config_mikes.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"

#include "../../../mikes-common/modules/passive/x_line_map.h"
#include "../../../mikes-common/modules/passive/pose.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/live/tim_hough_transform.h"

#include "../live/rect_localization.h"

#define MAXIMUM_RECT_LOCALIZATION_ATTEMPTS 5

typedef struct {
  int localize_is_required;
  int number_of_attampts;
  pthread_mutex_t data_lock;
} rect_map_t;

static rect_map_t rect_map_data;

void start_rect_map_localization()
{
  tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_CONTINUOUS);
}

void stop_rect_map_localization()
{
  tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_SINGLE);
}


int can_update_localization()
{
  pthread_mutex_lock(&rect_map_data.data_lock);

  if (rect_map_data.localize_is_required) {
    rect_map_data.localize_is_required = 0;
    stop_rect_map_localization();

    pthread_mutex_unlock(&rect_map_data.data_lock);
    return 1;
  }

  pthread_mutex_unlock(&rect_map_data.data_lock);
  return 0;
}

void unsuccessful_attempt()
{
  pthread_mutex_lock(&rect_map_data.data_lock);

  rect_map_data.localize_is_required = rect_map_data.localize_is_required + 1;

  if (rect_map_data.localize_is_required > MAXIMUM_RECT_LOCALIZATION_ATTEMPTS) {
    stop_rect_map_localization();
    printf("End with unsuccess localization\n");
  }

  pthread_mutex_unlock(&rect_map_data.data_lock);
}

void request_scan_and_localize()
{
  pthread_mutex_lock(&rect_map_data.data_lock);

  rect_map_data.number_of_attampts = 0;
  rect_map_data.localize_is_required = 1;
  start_rect_map_localization();

  pthread_mutex_unlock(&rect_map_data.data_lock);
}

void update_rect_localization(rect_localization_t *result)
{
  if (result->status == RECT_LOCALIZATION_SUCCESS) {
    pose_type copy_p;
    copy_p.x = result->pose.x / 10.0;
    copy_p.y = result->pose.y / 10.0;
    copy_p.heading = result->pose.heading;

    if (can_update_localization()) {
      set_pose(copy_p.x, copy_p.y, copy_p.heading);
      x_line_map_toggle_pose_visible(1);
    }
  } else {
    unsuccessful_attempt();
  }
}

void update_base_data(base_data_type *data)
{
  pose_type p;
  get_pose(&p);
  p.x = p.x + 11;
  p.y = p.y + 11;
  x_line_map_set_pose(p);
}

void init_rect_map_localization()
{
  pthread_mutex_init(&rect_map_data.data_lock, 0);
  register_base_callback(update_base_data);
  register_rect_localization_callback(update_rect_localization);
}

void shutdown_rect_map_localization()
{
  unregister_rect_localization_callback(update_rect_localization);
  unregister_base_callback(update_base_data);
}
