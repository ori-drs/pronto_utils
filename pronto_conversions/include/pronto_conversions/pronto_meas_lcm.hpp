#pragma once
#include <pronto_core/definitions.hpp>

#include <lcmtypes/bot_core/gps_data_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/bot_core/ins_t.hpp>
#include <lcmtypes/bot_core/six_axis_force_torque_array_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>

#include <lcmtypes/pronto/indexed_measurement_t.hpp>
#include <lcmtypes/pronto/filter_state_t.hpp>
#include <lcmtypes/pronto/update_t.hpp>


namespace MavStateEst {

void gpsDataFromLCM(const bot_core::gps_data_t& lcm_msg,
                    GPSMeasurement& msg);

void indexMeasurementFromLCM(const pronto::indexed_measurement_t& lcm_msg,
                             IndexedMeasurement& msg);

void filterStateFromLCM(const pronto::filter_state_t& lcm_msg,
                        FilterState& msg);

void poseMeasurementFromLCM(const bot_core::pose_t& lcm_msg,
                            PoseMeasurement& msg);

/**
 * @brief getVisualOdometryUpdateFromLCM takes an LCM message and a pose
 * and converts them into a VisualOdometryUpdate, which contains the time t0
 * the pose of the robot at t0, the time t1 and the relative pose between
 * t0 and t1, plus covariance and status.
 * @param[in] lcm_update
 * @param[in] body_to_local
 * @param[out] vo_update
 */
void getVisualOdometryUpdateFromLCM(const pronto::update_t& lcm_update,
                                      const Eigen::Affine3d& body_to_local,
                                      VisualOdometryUpdate& vo_update);

void rigidTransformFromLCM(const bot_core::rigid_transform_t &lcm_transf,
                           RigidTransform &transf);

void jointStateFromLCM(const bot_core::joint_state_t& lcm_msg,
                       JointState& msg);

void imuMeasurementFromLCM(const bot_core::ins_t& lcm_msg, ImuMeasurement& msg);

void forceTorqueFromLCM(const bot_core::six_axis_force_torque_array_t& lcm_msg,
                        ForceTorqueSensorArray& msg);

} // namespace MavStateEst
