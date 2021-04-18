#include "plan_env/sdf_map.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sdf");
    ros::NodeHandle node("~");
    fast_planner::SDFMap sdfMap;
    sdfMap.initMap(node);
    ros::spin();
    return 0;
}