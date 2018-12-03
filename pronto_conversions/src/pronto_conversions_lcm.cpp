#include "pronto_conversions/pronto_conversions_lcm.hpp"

namespace pronto
{
using MavStateEst::RBIS;
using MavStateEst::RBIM;
using Eigen::Map;
using std::vector;

Isometry3dTime getPoseAsIsometry3dTime(const bot_core::pose_t* pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  << pose->pos[0], pose->pos[1] , pose->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose->orientation[0], pose->orientation[1],
                                               pose->orientation[2], pose->orientation[3]);
  pose_iso.rotate(quat);
  return Isometry3dTime(pose->utime, pose_iso);
}

bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t tf;
  tf.utime = utime;
  tf.pos[0] = pose.translation().x();
  tf.pos[1] = pose.translation().y();
  tf.pos[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.orientation[0] = quat.w();
  tf.orientation[1] = quat.x();
  tf.orientation[2] = quat.y();
  tf.orientation[3] = quat.z();
  return tf;
}

bot_core::position_3d_t getIsometry3dAsPosition3d(Eigen::Isometry3d pose){
  bot_core::position_3d_t tf;
  tf.translation.x = pose.translation().x();
  tf.translation.y = pose.translation().y();
  tf.translation.z = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.rotation.w = quat.w();
  tf.rotation.x = quat.x();
  tf.rotation.y = quat.y();
  tf.rotation.z = quat.z();
  return tf;
}


// Scale a transform by elapsed time:
// was getTransAsVelocityTrans
Eigen::Isometry3d getDeltaAsVelocity(Eigen::Isometry3d delta, int64_t dt){

  Eigen::Quaterniond quat(delta.rotation());
  double rpy[3];
  quat_to_euler(quat, rpy[0], rpy[1], rpy[2]);

  // NB: I don't believe this is the correct way of calculating this:
  double elapsed_time = (double) dt*1E-6;// ( (double) utime -  prev_utime)/1000000;
  double rpy_rate[3];
  rpy_rate[0] = rpy[0]/elapsed_time;
  rpy_rate[1] = rpy[1]/elapsed_time;
  rpy_rate[2] = rpy[2]/elapsed_time;

  /*
  if (verbose){
    std::stringstream ss;
    ss << utime << " delta: ";
    printTrans(delta, ss.str() );
    std::cout << "Elapsed Time: " << elapsed_time  << " sec\n";
    std::cout << "RPY: " << rpy[0] << ", "<<rpy[1] << ", "<<rpy[2] <<" rad\n";
    std::cout << "RPY: " << rpy[0]*180/M_PI << ", "<<rpy[1]*180/M_PI << ", "<<rpy[2]*180/M_PI <<" deg\n";
    std::cout << "RPY: " << rpy_rate[0] << ", "
                         << rpy_rate[1] << ", "
                         << rpy_rate[2] << " rad/s | velocity scaled\n";
    std::cout << "RPY: " << rpy_rate[0]*180/M_PI << ", "
                         << rpy_rate[1]*180/M_PI << ", "
                         << rpy_rate[2]*180/M_PI << " deg/s | velocity scaled\n";
    std::cout << "XYZ: " << delta.trans_vec[0] << ", "
                         << delta.trans_vec[1] << ", "
                         << delta.trans_vec[2] << "\n";
  }
  */

  Eigen::Isometry3d vel;
  vel.setIdentity();
  vel.translation()  << delta.translation().x()/elapsed_time, delta.translation().y()/elapsed_time, delta.translation().z()/elapsed_time;
  Eigen::Quaterniond quat_vel = euler_to_quat(rpy_rate[0], rpy_rate[1], rpy_rate[2]);
  vel.rotate(quat_vel);

  /*
  if (verbose){
    std::stringstream ss2;
    ss2 << " vel: ";
    printTrans(vel, ss2.str() );
    std::cout << "\n\n";
  }
  */

  return vel;
}


// Given an Isometry3d representing velocity,
// inserts the translation components as the velocity components in the pose (for visualization in signal scope)
bot_core::pose_t getIsometry3dAsBotPoseVelocity(Eigen::Isometry3d pose_iso_velocity, int64_t utime ){
  bot_core::pose_t pose;
  pose.utime = utime;
  pose.pos[0] = 0;
  pose.pos[1] = 0;
  pose.pos[2] = 0;
  pose.orientation[0] = 0;
  pose.orientation[1] = 0;
  pose.orientation[2] = 0;
  pose.orientation[3] = 0;
  pose.vel[0] = pose_iso_velocity.translation().x();
  pose.vel[1] = pose_iso_velocity.translation().y();
  pose.vel[2] = pose_iso_velocity.translation().z();
  pose.rotation_rate[0] = 0;
  pose.rotation_rate[1] = 0;
  pose.rotation_rate[2] = 0;
  return pose;
}

MavStateEst::RBIS filterStateToRBIS(const pronto_filter_state_t * msg)
{
    MavStateEst::RBIS state(Eigen::Map<const Eigen::VectorXd>(msg->state, msg->num_states));
    if (msg->num_states != state.rbis_num_states) {
        fprintf(stderr, "error, constructed RBIS from rbis_filter_state_t of wrong size\n");
    }
    eigen_utils::botDoubleToQuaternion(state.quat, msg->quat);
    state.utime = msg->utime;
    return state;
}
MavStateEst::RBIS filterStateToRBIS(const pronto::filter_state_t & msg) {
    MavStateEst::RBIS state(Eigen::Map<const Eigen::VectorXd>(&msg.state[0], msg.num_states));
    if (msg.num_states != state.rbis_num_states) {
        fprintf(stderr, "error, constructed RBIS from rbis_filter_state_t of wrong size\n");
    }
    eigen_utils::botDoubleToQuaternion(state.quat, msg.quat);
    state.utime = msg.utime;
    return state;
}

pronto_filter_state_t * rbisCreateFilterStateMessage(const RBIS & state, const RBIM & cov){
    pronto_filter_state_t * msg = (pronto_filter_state_t *) malloc(sizeof(pronto_filter_state_t));

    msg->utime = state.utime;

    msg->num_states = RBIS::rbis_num_states;
    msg->num_cov_elements = msg->num_states * msg->num_states;
    eigen_utils::quaternionToBotDouble(msg->quat, state.quat);

    msg->state = (double *) malloc(msg->num_states * sizeof(double));
    msg->cov = (double *) malloc(msg->num_cov_elements * sizeof(double));

    Map<RBIS::VectorNd>(msg->state) = state.vec;
    Map<RBIM>(msg->cov) = cov;

  return msg;
}

pronto::filter_state_t rbisCreateFilterStateMessageCPP(const RBIS & state, const RBIM & cov){
    pronto::filter_state_t msg;
    msg.utime = state.utime;

    msg.num_states = RBIS::rbis_num_states;
    msg.num_cov_elements = msg.num_states * msg.num_states;
    eigen_utils::quaternionToBotDouble(msg.quat, state.quat);

    msg.state = vector<double>(msg.num_states);
    msg.cov = vector<double>(msg.num_cov_elements);

    Map<RBIS::VectorNd>(&msg.state[0]) = state.vec;
    Map<RBIM>(&msg.cov[0]) = cov;

  return msg;
}
void getPoseMsgFromState(const RBIS &state, bot_core::pose_t &msg)
{
    Eigen::Map<Eigen::Vector3d>(&msg.rotation_rate[0]) = state.angularVelocity();
    Eigen::Map<Eigen::Vector3d>(&msg.vel[0]) = state.velocity();
    Eigen::Map<Eigen::Vector3d>(&msg.pos[0]) = state.position();
    Eigen::Map<Eigen::Vector3d>(&msg.accel[0]) = state.acceleration();
    eigen_utils::quaternionToBotDouble(&msg.orientation[0], state.quat);
    msg.utime = state.utime;
}
}
namespace eigen_utils {
void  getPoseFromRigidBodyState(const RigidBodyState& state, eigen_utils_pose_t * pose)
{
  Eigen::Map<Eigen::Vector3d>(pose->rotation_rate) = state.angularVelocity();
  Eigen::Map<Eigen::Vector3d>(pose->vel) = state.velocity();
  Eigen::Map<Eigen::Vector3d>(pose->pos) = state.position();
  Eigen::Map<Eigen::Vector3d>(pose->accel) = state.acceleration();
  eigen_utils::quaternionToBotDouble(pose->orientation, state.quat);
  pose->utime = state.utime;
}

eigen_utils::pose_t getPoseFromRigidBodyState(const RigidBodyState& state)
{
  eigen_utils::pose_t pose;
  Eigen::Map<Eigen::Vector3d>(&pose.rotation_rate[0]) = state.angularVelocity();
  Eigen::Map<Eigen::Vector3d>(&pose.vel[0]) = state.velocity();
  Eigen::Map<Eigen::Vector3d>(&pose.pos[0]) = state.position();
  Eigen::Map<Eigen::Vector3d>(&pose.accel[0]) = state.acceleration();
  eigen_utils::quaternionToBotDouble(&pose.orientation[0], state.quat);
  pose.utime = state.utime;
  return pose;
}

RigidBodyState getRigidBodyStateFromPose(const eigen_utils_pose_t * pose)
{
    RigidBodyState state;
    Eigen::Map<const Eigen::Vector3d> velocity_map(pose->vel);
    Eigen::Map<const Eigen::Vector3d> angular_velocity_map(pose->rotation_rate);
    Eigen::Map<const Eigen::Vector3d> position_map(pose->pos);
    Eigen::Map<const Eigen::Vector3d> acceleration_map(pose->accel);

    state.velocity() = velocity_map;
    state.angularVelocity() = angular_velocity_map;
    state.position() = position_map;
    state.acceleration() = acceleration_map;
    state.chi() = Eigen::Vector3d::Zero();
    state.utime = pose->utime;

    eigen_utils::botDoubleToQuaternion(state.quat, pose->orientation);
    return state;
}

RigidBodyState getRigidBodyStateFromPose(const eigen_utils::pose_t * pose)
{
    RigidBodyState state;
    Eigen::Map<const Eigen::Vector3d> velocity_map(pose->vel);
    Eigen::Map<const Eigen::Vector3d> angular_velocity_map(pose->rotation_rate);
    Eigen::Map<const Eigen::Vector3d> position_map(pose->pos);
    Eigen::Map<const Eigen::Vector3d> acceleration_map(pose->accel);

    state.velocity() = velocity_map;
    state.angularVelocity() = angular_velocity_map;
    state.position() = position_map;
    state.acceleration() = acceleration_map;
    state.chi() = Eigen::Vector3d::Zero();
    state.utime = pose->utime;

    eigen_utils::botDoubleToQuaternion(state.quat, pose->orientation);
}

}
