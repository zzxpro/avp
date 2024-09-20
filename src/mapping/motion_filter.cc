#include "motion_filter.h"

namespace AVP
{
namespace mapping
{
    
// 判断位姿和上一次插入的位姿是否超过一定的范围，然后再决定是否插入scan。这个位姿是ceres返回的位姿
bool MotionFilter::IsSimilar(const Eigen::Vector3d& pose)
{
    double translation = std::sqrt(std::pow((pose[0]-last_pose_[0]),2)+std::pow((pose[1]-last_pose_[1]),2));
    double rotation = std::pow((pose[2]-last_pose_[2]),2);
    
    if (total_ > 0 && translation < 0.2 && rotation < 0.2)
    {
        // ROS_INFO("similar pose, ignore");
        return true;
    }
    
    last_pose_ = pose;
    total_++;
    return false;
}

} // namespace mapping
} // namespace AVP
