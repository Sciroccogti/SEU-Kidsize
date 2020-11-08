#include "skill.hpp"

#include <memory>

using namespace std;
using namespace Eigen;
using namespace seumath;

const double skill_goto_max_speed = 0.035;
const double skill_goto_stop_distance = 0.2;
const double skill_goto_stop_direction = 10.0;
const double skill_goto_turn_direction = 10.0;

common::BodyTask skill_goto(const common::PlayerInfo &pInfo,
                            const Eigen::Vector2d &target, double dir) {
    Vector2d target_in_self =
        target - Eigen::Vector2d(pInfo.self_x, pInfo.self_y);
    double dis = target_in_self.norm();
    common::BodyTask btask;
    btask.type = common::BodyTask::TASK_WALK;
    btask.count = 2;

    if (dis > skill_goto_stop_distance) {
        double azi_deg = azimuthDeg(target_in_self);
        double temp = normalizeDeg(azi_deg - pInfo.self_direction);
        bound(-skill_goto_turn_direction, skill_goto_turn_direction, temp);
        btask.step = skill_goto_max_speed;
        btask.turn = temp;
        return btask;
    } else if (fabs(pInfo.self_direction - dir) > skill_goto_stop_direction) {
        double temp_dir = normalizeDeg(dir - pInfo.self_direction);
        bound(-skill_goto_turn_direction, skill_goto_turn_direction, temp_dir);
        btask.turn = temp_dir;
        return btask;
    } else {
        btask.type = common::BodyTask::TASK_ACT;
        btask.actname = "ready";
        return btask;
    }
}

common::BodyTask skill_penalty_kick(common::ObjInfo ball) {
    const float width = 640;
    const float height = 480;
    float alpha = (ball.x + ball.w / 2) / width - 0.5;
    float beta = (ball.y + ball.h / 2) / height - 0.5;
    common::BodyTask btask;
    btask.type = common::BodyTask::TASK_WALK;
    btask.count = 2;
    if (alpha > -0.05) {
        btask.lateral = -0.01;
        return btask;
    } else if (alpha < -0.15) {
        btask.lateral = 0.01;
        return btask;
    } else {
        if (beta < 0.32) {
            btask.step = 0.01;
            return btask;
        } else if (beta > 0.4) {
            btask.step = -0.01;
            return btask;
        } else {
            btask.type = common::BodyTask::TASK_ACT;
            btask.actname = "left_little_kick";
            return btask;
        }
    }
}