#include "plan_env/sdf_map.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace fast_planner {
    class TestSDFNodelet : public nodelet::Nodelet {
    public:
        TestSDFNodelet()=default;
        virtual void onInit() {
            ros::NodeHandle& nh = getPrivateNodeHandle();
            sdfMap.initMap(nh);
        }
        fast_planner::SDFMap sdfMap;
    };
}

PLUGINLIB_EXPORT_CLASS(fast_planner::TestSDFNodelet, nodelet::Nodelet)