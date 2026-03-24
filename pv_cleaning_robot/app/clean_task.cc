#include "pv_cleaning_robot/app/clean_task.h"
#include <cmath>

namespace robot::app {

CleanTask::CleanTask(std::shared_ptr<service::MotionService> motion,
                     std::shared_ptr<service::NavService>    nav,
                     Config                                  cfg)
    : motion_(std::move(motion))
    , nav_(std::move(nav))
    , cfg_(cfg)
{
}

bool CleanTask::start()
{
    if (!paused_) return true;  // 已在运行

    pass_start_distance_ = static_cast<float>(nav_->get_pose().distance_m);
    paused_.store(false);
    return motion_->start_cleaning();
}

void CleanTask::pause()
{
    paused_.store(true);
    motion_->stop_cleaning();
}

void CleanTask::stop()
{
    paused_.store(true);
    pass_count_     = 0;
    going_forward_  = true;
    motion_->stop_cleaning();
}

bool CleanTask::reached_end() const
{
    if (paused_) return false;
    auto pose = nav_->get_pose();
    float travelled = static_cast<float>(pose.distance_m) - pass_start_distance_;

    if (going_forward_) {
        return travelled >= cfg_.track_length_m;
    } else {
        return travelled <= 0.0f;
    }
}

bool CleanTask::all_passes_done() const
{
    return pass_count_ >= cfg_.passes;
}

void CleanTask::switch_direction()
{
    going_forward_ = !going_forward_;
    ++pass_count_;
    pass_start_distance_ = static_cast<float>(nav_->get_pose().distance_m);

    if (going_forward_) {
        motion_->start_cleaning();
    } else {
        // 反向：负速度
        motion_->set_walk_speed(-300.0f);
    }
}

} // namespace robot::app
