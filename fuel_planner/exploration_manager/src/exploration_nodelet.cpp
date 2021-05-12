#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <pluginlib/class_list_macros.h>

#include <plan_manage/backward.hpp>
namespace backward {
    backward::SignalHandling sh;
}

namespace fast_planner {
    class ExplorationNodelet : public nodelet::Nodelet {
    public:
        virtual void onInit() {
            ros::NodeHandle& nh = getNodeHandle();
            FastExplorationFSM expl_fsm;
            expl_fsm.init(nh);
            ros::Duration(1.0).sleep();
        }
    };
}

PLUGINLIB_EXPORT_CLASS(fast_planner::ExplorationNodelet, nodelet::Nodelet)
