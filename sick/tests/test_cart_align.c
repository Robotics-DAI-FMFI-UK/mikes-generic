#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include "../../mikes-common/modules/live/tim571.h"
#include "../../mikes-common/modules/live/base_module.h"
#include "../../mikes-common/modules/passive/pose.h"
#include "../modules/live/sick_cart_align.h"
#include "../../mikes-common/bites/mikes.h"

void test_cart_align()
{
    align_robot_to_cart();
}

int main(int argc, char **argv)
{
    mikes_init(argc, argv);
    init_pose(0, 7000);
    init_base_module();

    init_tim571();
    init_sick_cart_align();

    sleep(1);
    test_cart_align();

    while (program_runs)
    {
      usleep(100000);
    }

    mikes_shutdown();

    return 0;
}
