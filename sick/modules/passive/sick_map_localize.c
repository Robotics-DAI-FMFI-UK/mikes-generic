#include <unistd.h>
#include <stdio.h>

#include "core/config_mikes.h"
#include "../../../mikes-common/modules/passive/mikes_logs.h"

#include "../../../mikes-common/modules/passive/x_line_map.h"
#include "../../../mikes-common/modules/passive/pose.h"
#include "../../../mikes-common/modules/live/base_module.h"
#include "../../../mikes-common/modules/live/navig.h"
#include "../../../mikes-common/modules/live/tim_hough_transform.h"

#include "../live/sick_localization.h"

#define WAIT_BASE_DATA 2
#define WAIT_INITIAL_FIND_POSITION 2

#define SML_LOGSTR_LEN 1024

static int is_finding_initial_pose = 0;

int can_update_pose()
{
  return is_finding_initial_pose || navig_can_actualize_pose_now();
}

void update_sick_localization(sick_localization_t *result)
{
  if (result->status == SICK_LOCALIZATION_SUCCESS) {
    pose_type copy_p;
    copy_p.x = result->pose.x / 10.0;
    copy_p.y = result->pose.y / 10.0;
    copy_p.heading = result->pose.heading;

    pose_type old_p;
    get_pose(&old_p);

    base_data_type base_data;
    get_base_data(&base_data);

    char str[SML_LOGSTR_LEN];

    if (can_update_pose()) {
      set_pose(copy_p.x, copy_p.y, copy_p.heading);

      sprintf(str, "[main] sick_map_localize::update_sick_localization_updated(): x=%0.2f, y=%0.2f, heading_deg=%0.2f, old_x=%0.2f, old_y=%0.2f, old_heading_deg=%0.2f, base_heading=%d",
        copy_p.x, copy_p.y, copy_p.heading / M_PI * 180.0, old_p.x, old_p.y, old_p.heading / M_PI * 180.0, (int)base_data.heading);
      mikes_log(ML_DEBUG, str);
    } else {
      sprintf(str, "[main] sick_map_localize::update_sick_localization_not_updated(): x=%0.2f, y=%0.2f, heading_deg=%0.2f, old_x=%0.2f, old_y=%0.2f, old_heading_deg=%0.2f, base_heading=%d",
        copy_p.x, copy_p.y, copy_p.heading / M_PI * 180.0, old_p.x, old_p.y, old_p.heading / M_PI * 180.0, (int)base_data.heading);
      mikes_log(ML_DEBUG, str);
    }
  } else {
    navig_fail_actualize_pose();
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

void use_config_initial_localization()
{
  x_line_map_toggle_pose_visible(1);
  set_pose(mikes_config.localization_base_x, mikes_config.localization_base_y, mikes_config.localization_base_heading * M_PI / 180.0);

  pose_type initial_x_line_pose;
  initial_x_line_pose.x = mikes_config.localization_base_x + 11;
  initial_x_line_pose.y = mikes_config.localization_base_y + 11;
  initial_x_line_pose.heading = mikes_config.localization_base_heading * M_PI / 180.0;
  x_line_map_set_pose(initial_x_line_pose);
}

void init_sick_map_localize()
{
  use_config_initial_localization();
  register_base_callback(update_base_data);
  register_sick_localization_callback(update_sick_localization);
}

void shutdown_sick_map_localize()
{
  unregister_sick_localization_callback(update_sick_localization);
  unregister_base_callback(update_base_data);
}

void find_starting_localization()
{
  sleep(WAIT_BASE_DATA);

  is_finding_initial_pose = 1;
  tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_CONTINUOUS);
  sleep(WAIT_INITIAL_FIND_POSITION);
  tim_hough_transform_set_mode(TIM_HOUGH_TRANSFORM_MODE_SINGLE);
  is_finding_initial_pose = 0;
}
