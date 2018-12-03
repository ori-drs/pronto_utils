#include <pronto_core/definitions.hpp>
#include <eigen_utils/eigen_utils.hpp>
#include "pronto_conversions/pronto_meas_lcm.hpp"

namespace MavStateEst {

void gpsDataFromLCM(const bot_core::gps_data_t &lcm_msg,
                    GPSMeasurement &msg)
{
    msg.elev = lcm_msg.elev;
    msg.gps_lock = lcm_msg.gps_lock;
    msg.gps_time = lcm_msg.gps_time;
    msg.heading = lcm_msg.heading;
    msg.horizontal_accuracy = lcm_msg.horizontal_accuracy;
    msg.latitude = lcm_msg.latitude;
    msg.longitude = lcm_msg.longitude;
    msg.numSatellites = lcm_msg.numSatellites;
    msg.speed = lcm_msg.speed;
    msg.utime = lcm_msg.utime;
    msg.vertical_accuracy = lcm_msg.vertical_accuracy;
    msg.xyz_pos = Eigen::Map<const Eigen::Vector3d>(lcm_msg.xyz_pos);
}

void indexMeasurementFromLCM(const pronto::indexed_measurement_t &lcm_msg,
                             IndexedMeasurement &msg)
{
    msg.R_effective = Eigen::Map<const Eigen::MatrixXd>(&lcm_msg.R_effective[0],
                                                        lcm_msg.measured_dim,
                                                        lcm_msg.measured_dim);
    msg.state_utime = lcm_msg.state_utime;
    msg.utime = lcm_msg.utime;
    msg.z_effective = Eigen::Map<const Eigen::VectorXd>(&lcm_msg.z_effective[0],
                                                         lcm_msg.measured_dim);
    msg.z_indices = Eigen::Map<const Eigen::VectorXi>(&lcm_msg.z_indices[0],
                                                       lcm_msg.measured_dim);
}

void filterStateFromLCM(const pronto::filter_state_t &lcm_msg,
                        FilterState &msg)
{
    Eigen::Quaterniond init_quat;
    eigen_utils::botDoubleToQuaternion(init_quat, lcm_msg.quat);
    Eigen::Map<const Eigen::MatrixXd> cov_map(&lcm_msg.cov[0], lcm_msg.num_states, lcm_msg.num_states);
    msg.cov = cov_map;
    msg.quat = init_quat;
    msg.state = Eigen::Map<const Eigen::VectorXd>(&lcm_msg.state[0], lcm_msg.num_states);
    msg.utime = lcm_msg.utime;
}

void jointStateFromLCM(const bot_core::joint_state_t &lcm_msg,
                       JointState &msg)
{
    // std::vector copy should work here
    msg.utime = lcm_msg.utime;
    msg.joint_name = lcm_msg.joint_name;
    // using interator constructors because lcm_msg has vector<float> and
    // msg has vector<double>
    msg.joint_position = std::vector<double>(lcm_msg.joint_position.begin(),
                                             lcm_msg.joint_position.end());
    msg.joint_velocity = std::vector<double>(lcm_msg.joint_velocity.begin(),
                                             lcm_msg.joint_velocity.end());
    msg.joint_effort = std::vector<double>(lcm_msg.joint_effort.begin(),
                                           lcm_msg.joint_effort.end());
    // there is no acceleration in the LCM message
    msg.joint_acceleration = std::vector<double>(msg.joint_effort.size(), 0.0);
}

void imuMeasurementFromLCM(const bot_core::ins_t& lcm_msg, ImuMeasurement& msg) {
    msg.acceleration = Eigen::Map<const Eigen::Vector3d>(lcm_msg.accel);
    msg.utime = lcm_msg.utime;
    msg.omega = Eigen::Map<const Eigen::Vector3d>(lcm_msg.gyro);
}

void forceTorqueFromLCM(const bot_core::six_axis_force_torque_array_t &lcm_msg,
                        ForceTorqueSensorArray &msg)
{
    msg.utime = lcm_msg.utime;
    msg.num_sensors = lcm_msg.num_sensors;
    msg.sensors.resize(lcm_msg.num_sensors);
    ForceTorqueSensor ftsensor;
    for(int i = 0; i < msg.num_sensors; i++){
        msg.names[i] = lcm_msg.names[i];
        for(int j = 0; j < 3; j++){
            ftsensor.force[j] =  lcm_msg.sensors[i].force[j];
            ftsensor.moment[j] = lcm_msg.sensors[i].moment[j];
        }
        msg.sensors[i] = ftsensor;
    }
}

void getVisualOdometryUpdateFromLCM(const pronto::update_t &lcm_update,
                                    const Eigen::Affine3d &body_to_local,
                                    VisualOdometryUpdate &vo_update)
{
    vo_update.curr_utime = lcm_update.timestamp;
    vo_update.pose_covariance = Eigen::Map<const Eigen::Matrix<double,6,6, Eigen::RowMajor>>(&lcm_update.covariance[0][0]);

    vo_update.prev_pose = body_to_local;
    vo_update.prev_utime = lcm_update.prev_timestamp;
    vo_update.relative_pose.setIdentity();
    vo_update.relative_pose.translate(Eigen::Map<const Eigen::Vector3d>(lcm_update.translation));
    Eigen::Quaterniond temp_quat = Eigen::Quaterniond::Identity();
    temp_quat.w() = lcm_update.rotation[0];
    temp_quat.x() = lcm_update.rotation[1];
    temp_quat.y() = lcm_update.rotation[2];
    temp_quat.z() = lcm_update.rotation[3];
    vo_update.relative_pose.rotate(temp_quat);
    vo_update.status = static_cast<VisualOdometryUpdate::Status>(lcm_update.estimate_status);
}

void poseMeasurementFromLCM(const bot_core::pose_t &lcm_msg,
                            PoseMeasurement &msg)
{
    // convert LCM to generic format
    msg.utime = lcm_msg.utime;
    msg.linear_vel = Eigen::Map<const Eigen::Vector3d>(lcm_msg.vel);
    msg.angular_vel = Eigen::Map<const Eigen::Vector3d>(lcm_msg.rotation_rate);
    msg.pos = Eigen::Map<const Eigen::Vector3d>(lcm_msg.pos);
    eigen_utils::botDoubleToQuaternion(msg.orientation, lcm_msg.orientation);
}

void rigidTransformFromLCM(const bot_core::rigid_transform_t &lcm_transf,
                           RigidTransform &transf)
{
    transf.transform = Transform::Identity();
    transf.transform.translate(Eigen::Map<const Eigen::Vector3d>(lcm_transf.trans));

    Eigen::Quaterniond quat;
    quat.w() = lcm_transf.quat[0];
    quat.x() = lcm_transf.quat[1];
    quat.y() = lcm_transf.quat[2];
    quat.z() = lcm_transf.quat[3];
    transf.transform.rotate(quat);
    transf.utime = lcm_transf.utime;
}

} // namespace MavStateEst




