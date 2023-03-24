#include "ros/ros.h"
#include <iostream>

#include "gridcell.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridcell");

    Gridcell_map gridcell_map;

    // init
    gridcell_map.gridcellInit();

    //ros::spin();
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        gridcell_map.pubcell();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
