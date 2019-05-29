#include <stdio.h>

#include "config_mikes.h"
#include "../mikes-common/bites/mikes.h"

mikes_config_t mikes_config;

mikes_config_t default_mikes_config = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
mikes_config_t mikes_config;

void load_config()
{
    config_data cfg = read_config(MIKES_CONFIG);
    if (cfg == 0)
    {
       printf("Could not open config file %s\n", MIKES_CONFIG);
       program_runs = 0;
    }
    mikes_config = default_mikes_config;
    mikes_config.autostart = config_get_intval(cfg, "autostart", mikes_config.autostart);
    mikes_config.with_gui = config_get_intval(cfg, "show_gui", mikes_config.with_gui);
    mikes_config.start_x = config_get_intval(cfg, "start_x", mikes_config.start_x);
    mikes_config.start_y = config_get_intval(cfg, "start_y", mikes_config.start_y);

    mikes_config.print_all_logs_to_console = config_get_intval(cfg, "print_all_logs_to_console",
                                                               mikes_config.print_all_logs_to_console);
    mikes_config.print_debug_logs = config_get_intval(cfg, "print_debug_logs", mikes_config.print_debug_logs);

    mikes_config.use_ncurses_control = config_get_intval(cfg, "use_ncurses_control", mikes_config.use_ncurses_control);

    mikes_config.use_rplidar = config_get_intval(cfg, "use_rplidar", mikes_config.use_rplidar);
    mikes_config.use_ust10lx = config_get_intval(cfg, "use_ust10lx", mikes_config.use_ust10lx);
    mikes_config.use_tim571 = config_get_intval(cfg, "use_tim571", mikes_config.use_tim571);
    mikes_config.use_xtion = config_get_intval(cfg, "use_xtion", mikes_config.use_xtion);
    mikes_config.use_rfid = config_get_intval(cfg, "use_rfid", mikes_config.use_rfid);

    mikes_config.use_tim_hough_transform = config_get_intval(cfg, "use_tim_hough_transform", mikes_config.use_tim_hough_transform);
    mikes_config.use_line_filter = config_get_intval(cfg, "use_line_filter", mikes_config.use_line_filter);
    mikes_config.use_tim_segment = config_get_intval(cfg, "use_tim_segment", mikes_config.use_tim_segment);
    mikes_config.use_tim_corner = config_get_intval(cfg, "use_tim_corner", mikes_config.use_tim_corner);

    mikes_config.map_azimuth = config_get_doubleval(cfg, "map_azimuth", mikes_config.map_azimuth);

    mikes_config.line_map_file = config_get_alloc_strval(cfg, "line_map_file", "file_missing_in_config");
    mikes_config.xtion_samples_config = config_get_alloc_strval(cfg, "xtion_samples_config", "file_missing_in_config");

    mikes_config.use_sick_localization = config_get_intval(cfg, "use_sick_localization", mikes_config.use_sick_localization);
    mikes_config.use_sick_cart_align = config_get_intval(cfg, "use_sick_cart_align", mikes_config.use_sick_cart_align);
    mikes_config.use_sick_strategy = config_get_intval(cfg, "use_sick_strategy", mikes_config.use_sick_strategy);

    mikes_config.localization_start_x = config_get_doubleval(cfg, "localization_start_x", mikes_config.localization_start_x);
    mikes_config.localization_start_y = config_get_doubleval(cfg, "localization_start_y", mikes_config.localization_start_y);
    mikes_config.localization_start_heading = config_get_doubleval(cfg, "localization_start_heading", mikes_config.localization_start_heading);

    mikes_config.localization_waiting1_x = config_get_doubleval(cfg, "localization_waiting1_x", mikes_config.localization_waiting1_x);
    mikes_config.localization_waiting1_y = config_get_doubleval(cfg, "localization_waiting1_y", mikes_config.localization_waiting1_y);
    mikes_config.localization_waiting1_heading = config_get_doubleval(cfg, "localization_waiting1_heading", mikes_config.localization_waiting1_heading);

    mikes_config.localization_waiting2_x = config_get_doubleval(cfg, "localization_waiting2_x", mikes_config.localization_waiting2_x);
    mikes_config.localization_waiting2_y = config_get_doubleval(cfg, "localization_waiting2_y", mikes_config.localization_waiting2_y);
    mikes_config.localization_waiting2_heading = config_get_doubleval(cfg, "localization_waiting2_heading", mikes_config.localization_waiting2_heading);

    mikes_config.localization_base1_x = config_get_doubleval(cfg, "localization_base1_x", mikes_config.localization_base1_x);
    mikes_config.localization_base1_y = config_get_doubleval(cfg, "localization_base1_y", mikes_config.localization_base1_y);
    mikes_config.localization_base1_heading = config_get_doubleval(cfg, "localization_base1_heading", mikes_config.localization_base1_heading);

    mikes_config.localization_base2_x = config_get_doubleval(cfg, "localization_base2_x", mikes_config.localization_base2_x);
    mikes_config.localization_base2_y = config_get_doubleval(cfg, "localization_base2_y", mikes_config.localization_base2_y);
    mikes_config.localization_base2_heading = config_get_doubleval(cfg, "localization_base2_heading", mikes_config.localization_base2_heading);

    mikes_config.localization_base3_x = config_get_doubleval(cfg, "localization_base3_x", mikes_config.localization_base3_x);
    mikes_config.localization_base3_y = config_get_doubleval(cfg, "localization_base3_y", mikes_config.localization_base3_y);
    mikes_config.localization_base3_heading = config_get_doubleval(cfg, "localization_base3_heading", mikes_config.localization_base3_heading);

    config_dispose(cfg);
}
