#ifndef PRONTO_CONVERSIONS_LCM_HPP_
#define PRONTO_CONVERSIONS_LCM_HPP_

#include <pronto_math/pronto_math.hpp>
#include <pronto_core/rbis.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/position_3d_t.hpp>
#include <lcmtypes/pronto/filter_state_t.hpp>
#include <lcmtypes/pronto_filter_state_t.h>
#include <lcmtypes/eigen_utils_pose_t.h>
#include <lcmtypes/eigen_utils/pose_t.hpp>

namespace pronto
{
using MavStateEst::RBIS;
using MavStateEst::RBIM;
using Eigen::Map;
using std::vector;

Isometry3dTime getPoseAsIsometry3dTime(const bot_core::pose_t* pose);

bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime);

bot_core::position_3d_t getIsometry3dAsPosition3d(Eigen::Isometry3d pose);

void getPoseMsgFromState(const RBIS &state, bot_core::pose_t &msg);


// Scale a transform by elapsed time:
// was getTransAsVelocityTrans
Eigen::Isometry3d getDeltaAsVelocity(Eigen::Isometry3d delta, int64_t dt);


// Given an Isometry3d representing velocity,
// inserts the translation components as the velocity components in the pose (for visualization in signal scope)
bot_core::pose_t getIsometry3dAsBotPoseVelocity(Eigen::Isometry3d pose_iso_velocity, int64_t utime );

MavStateEst::RBIS filterStateToRBIS(const pronto_filter_state_t * msg);

pronto_filter_state_t * rbisCreateFilterStateMessage(const RBIS & state, const RBIM & cov);

pronto::filter_state_t rbisCreateFilterStateMessageCPP(const RBIS & state, const RBIM & cov);

}
namespace eigen_utils {
void getPoseFromRigidBodyState(const RigidBodyState& state, eigen_utils_pose_t * pose);

eigen_utils::pose_t getPoseFromRigidBodyState(const RigidBodyState& state);

RigidBodyState getRigidBodyStateFromPose(const eigen_utils_pose_t * pose);

RigidBodyState getRigidBodyStateFromPose(const eigen_utils::pose_t * pose);

}


#endif /* PRONTO_CONVERSIONS_LCM_HPP_ */
