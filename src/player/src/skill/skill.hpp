#ifndef __SKILL_HPP
#define __SKILL_HPP

// #include "model.hpp"
// #include "task/task.hpp"

#include <common/BodyTask.h>
#include <common/ObjInfo.h>
#include <common/PlayerInfo.h>

#include <seumath/math.hpp>

extern common::BodyTask skill_goto(const common::PlayerInfo &self,
                                   const Eigen::Vector2d &target, double dir);
extern common::BodyTask skill_penalty_kick(common::ObjInfo ball);

#endif
