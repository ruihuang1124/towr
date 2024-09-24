/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>


namespace towr {


TowrRosInterface::TowrRosInterface ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRosInterface::UserCommandCallback, this);

  initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                          (xpp_msgs::robot_state_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);

  planned_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
            (xpp_msgs::robot_trajectory_desired, 1);

  robot_state_sub_ = n.subscribe(towr_msgs::robot_states_name, 1,
                                    &TowrRosInterface::RobotStatesCallback, this);

  solver_ = std::make_shared<ifopt::IpoptSolver>();
  robot_states_msgs_.body.pose.position.z = 0.29;
  robot_states_msgs_.body.pose.orientation.w = 1.0;

  visualization_dt_ = 0.01;
}

BaseState
TowrRosInterface::GetGoalState(const TowrCommandMsg& msg) const
{
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kPos).x() += robot_states_msgs_.body.pose.position.x;
  goal.lin.at(kPos).y() += robot_states_msgs_.body.pose.position.y;
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
    xpp::Quaterniond q;
    q.x() = robot_states_msgs_.body.pose.orientation.x;
    q.y() = robot_states_msgs_.body.pose.orientation.y;
    q.z() = robot_states_msgs_.body.pose.orientation.z;
    q.w() = robot_states_msgs_.body.pose.orientation.w;
  Vector3d euler = xpp::GetEulerZYXAngles(q);
  goal.ang.at(kPos).z() += euler[0];
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);
  return goal;
}

void
TowrRosInterface::UserCommandCallback(const TowrCommandMsg& msg)
{
  // robot model
  formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
  auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
  robot_parameters_pub_.publish(robot_params_msg);

  // terrain
  auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
  formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);

  int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
  formulation_.params_ = GetTowrParameters(n_ee, msg);
  formulation_.final_base_ = GetGoalState(msg);

  SetTowrInitialState();

//    if (msg.robot == 4){ // mini_cheetah
//        ROS_WARN("MiniCheetah!");
//        SetActualInitialState();
//    }

  // solver parameters
  SetIpoptParameters(msg);

  // visualization
  PublishInitialState();

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  std::string state_pose_csv_file = "state_post.csv";
  std::string contact_post_csv_file = "contact_post.csv";
    if (msg.optimize || msg.play_initialization) {
    nlp_ = ifopt::Problem();
    for (auto c : formulation_.GetVariableSets(solution))
      nlp_.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
      nlp_.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts())
      nlp_.AddCostSet(c);

    solver_->Solve(nlp_);
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
    if (msg.robot == 4){ // mini_cheetah
        ROS_WARN("MiniCheetah!");
        SaveOptimizationAsCSV(state_pose_csv_file, contact_post_csv_file, robot_params_msg, msg, false);
    }
  }

  // playback using terminal commands
  if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
    int success = system(("rosbag play --topics "
        + xpp_msgs::robot_state_desired + " "
        + xpp_msgs::terrain_info
        + " -r " + std::to_string(msg.replay_speed)
        + " --quiet " + bag_file).c_str());
  }

  if (msg.plot_trajectory) {
    int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
  }

  // to publish entire trajectory (e.g. to send to controller)
//  double t = 0.0;
//    solution.base_angular_->GetPoint(t);
  xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());
  planned_trajectory_pub_.publish(xpp_msg);
}

void TowrRosInterface::RobotStatesCallback(const towr::TowrRosInterface::TowrRobotStates &msg) {
//    ROS_WARN("Received robot states!");
    robot_states_msgs_ = msg;
}

void
TowrRosInterface::PublishInitialState()
{
  int n_ee = formulation_.initial_ee_W_.size();
  xpp::RobotStateCartesian xpp(n_ee);
  xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
  xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());

  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
    xpp.ee_contact_.at(ee_xpp)   = true;
    xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface::XppVec
TowrRosInterface::GetTrajectory () const
{
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

    void TowrRosInterface::GetTrajectoryAng() {
        states_angular_roll_.clear();
        states_angular_pitch_.clear();
        states_angular_yaw_.clear();
        double t = 0.0;
        double T = solution.base_linear_->GetTotalTime();
        EulerConverter base_angular(solution.base_angular_);
        while (t <= T + 1e-5) {
            auto states_ang = ToXpp(solution.base_angular_->GetPoint(t));
            states_angular_roll_.push_back(states_ang.p_[0]);
            states_angular_pitch_.push_back(states_ang.p_[1]);
            states_angular_yaw_.push_back(states_ang.p_[2]);
            t += visualization_dt_;
        }

    }

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRosInterface::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

    void TowrRosInterface::SaveOptimizationAsCSV(const std::string &state_post_csv_file,
                                                 const std::string &contact_post_csv_file,
                                                 const xpp_msgs::RobotParameters &robot_params,
                                                 const towr::TowrRosInterface::TowrCommandMsg user_command_msg,
                                                 bool include_iterations) {
        // get the optimized trajectory first
        auto final_trajectory = GetTrajectory();
        GetTrajectoryAng();
        int row = final_trajectory.size();

        // analise the trajectory and get the horizons vector
        std::vector<int> horizons;
        int horizon = 1;
        for (int i = 0; i < row - 1; i = i + 1) {
            std::vector<int> current_contact, next_contact;
            for (int leg = 0; leg < 4; ++leg) {
                int contact_current = 0, contact_next = 0;
                if (final_trajectory.at(i).ee_contact_.at(leg) == 1) {
                    contact_current = 1;
                }
                if (final_trajectory.at(i + 1).ee_contact_.at(leg) == 1) {
                    contact_next = 1;
                }
                current_contact.push_back(contact_current);
                next_contact.push_back(contact_next);
            }
            if (current_contact.at(0) == next_contact.at(0) && current_contact.at(1) == next_contact.at(1) &&
                current_contact.at(2) == next_contact.at(2) && current_contact.at(3) == next_contact.at(3)) {
                horizon += 1;
            } else {
                horizons.push_back(horizon);
                horizon = 1;
            }
            if (i == row - 2) {
                horizons.push_back(horizon);
            }
        }

        // Save the contact file first
        std::ofstream dataFile;
        dataFile.open(contact_post_csv_file, std::ios::out | std::ios::trunc);
        dataFile << "FR" << "," << "RL" << "," << "HR" << "," << "HL" << "," << "startTime" << "," << "endTime" << ","
                 << "horizon" << std::endl;
        int current_trajectory_point_index_for_contact = 0;
        for (int i = 0; i < horizons.size(); i++) {
            current_trajectory_point_index_for_contact += horizons.at(i);
            dataFile << final_trajectory.at(current_trajectory_point_index_for_contact - 1).ee_contact_.at(RF) << ",";
            dataFile << final_trajectory.at(current_trajectory_point_index_for_contact - 1).ee_contact_.at(LF) << ",";
            dataFile << final_trajectory.at(current_trajectory_point_index_for_contact - 1).ee_contact_.at(RH) << ",";
            dataFile << final_trajectory.at(current_trajectory_point_index_for_contact - 1).ee_contact_.at(LH) << ",";
            dataFile << final_trajectory.at(current_trajectory_point_index_for_contact - horizons.at(i)).t_global_
                     << ",";
            if (i == horizons.size() - 1) {
                dataFile << final_trajectory.at(current_trajectory_point_index_for_contact - 1).t_global_ + 0.01 << ",";
            } else {
                dataFile << final_trajectory.at(current_trajectory_point_index_for_contact).t_global_ << ",";
            }

            dataFile << horizons.at(i) << ",";
            dataFile << std::endl;
        }
        dataFile.close();

        // Save the optimized states as csv file, swing leg's joints are set as default values.
        dataFile.open(state_post_csv_file, std::ios::out | std::ios::trunc);
//        dataFile << "phase" << "," << "roll" << "," << "pitch" << "," << "yaw" << "," << "x" << "," << "y" << "," << "z" << ","
//                 << "omega_x"
//                 << "," << "omega_y" << "," << "omega_z" << "," << "v_x" << "," << "v_y" << "," << "v_z" << ","<< "q_dummy_1"
//                 << "," << "q_dummy_2" << "," << "q_dummy_3" << "," << "q_dummy_4" << "," << "q_dummy_5" << ","
//                 << "q_dummy_6" << ","
//                 << "q_dummy_7"
//                 << "," << "q_dummy_8" << "," << "q_dummy_9" << "," << "q_dummy_10" << "," << "q_dummy_11" << ","
//                 << "q_dummy_12" << std::endl;
//        for (int i = 0; i < row; i = i + 1) {
//            dataFile << states_angular_roll_.at(i) << ",";
//            dataFile << states_angular_pitch_.at(i) << ",";
//            dataFile << states_angular_yaw_.at(i) << ",";
//            dataFile << final_trajectory.at(i).base_.lin.p_[0] << ",";
//            dataFile << final_trajectory.at(i).base_.lin.p_[1] << ",";
//            dataFile << final_trajectory.at(i).base_.lin.p_[2] << ",";
//            dataFile << final_trajectory.at(i).base_.ang.w[0] << ",";
//            dataFile << final_trajectory.at(i).base_.ang.w[1] << ",";
//            dataFile << final_trajectory.at(i).base_.ang.w[2] << ",";
//            dataFile << final_trajectory.at(i).base_.lin.v_[0] << ",";
//            dataFile << final_trajectory.at(i).base_.lin.v_[1] << ",";
//            dataFile << final_trajectory.at(i).base_.lin.v_[2] << ",";
//            dataFile << std::endl;
//        }

        int current_trajectory_point_index = 0, current_phase_start_index = 0;
        for (int i = 0; i < horizons.size(); i++) {     // loop through all phases
            for (int j = 0; j <= horizons.at(i); ++j) {  // loop inside phase.
                if (i == 0){
                    current_trajectory_point_index = current_phase_start_index + j;
                } else{
                    current_trajectory_point_index = current_phase_start_index + j - 1;
                }
                dataFile << i << ",";
                dataFile << states_angular_roll_.at(current_trajectory_point_index) << ",";
                dataFile << states_angular_pitch_.at(current_trajectory_point_index) << ",";
                dataFile << states_angular_yaw_.at(current_trajectory_point_index) << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.lin.p_[0] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.lin.p_[1] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.lin.p_[2] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.ang.w[0]
                         << ","; //TODO w or w_world?
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.ang.w[1] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.ang.w[2] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.lin.v_[0] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.lin.v_[1] << ",";
                dataFile << final_trajectory.at(current_trajectory_point_index).base_.lin.v_[2] << ",";

                (final_trajectory.at(current_phase_start_index).ee_contact_.at(RF) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(RF).p_[0] << "," :
                dataFile
                        << 0.0 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(RF) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(RF).p_[1] << "," :
                dataFile
                        << -0.8 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(RF) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(RF).p_[2] << "," :
                dataFile
                        << 1.6 << ",";

                (final_trajectory.at(current_phase_start_index).ee_contact_.at(LF) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(LF).p_[0] << "," :
                dataFile
                        << 0.0 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(LF) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(LF).p_[1] << "," :
                dataFile
                        << -0.8 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(LF) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(LF).p_[2] << "," :
                dataFile
                        << 1.6 << ",";

                (final_trajectory.at(current_phase_start_index).ee_contact_.at(RH) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(RH).p_[0] << "," :
                dataFile
                        << 0.0 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(RH) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(RH).p_[1] << "," :
                dataFile
                        << -0.8 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(RH) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(RH).p_[2] << "," :
                dataFile
                        << 1.6 << ",";

                (final_trajectory.at(current_phase_start_index).ee_contact_.at(LH) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(LH).p_[0] << "," :
                dataFile
                        << 0.0 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(LH) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(LH).p_[1] << "," :
                dataFile
                        << -0.8 << ",";
                (final_trajectory.at(current_phase_start_index).ee_contact_.at(LH) == 1) ? dataFile
                        << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(LH).p_[2] << "," :
                dataFile
                        << 1.6 << ",";
//                for (int leg = 0; leg < 4; ++leg) {
//                    (final_trajectory.at(current_trajectory_point_index).ee_contact_.at(leg) == 1) ? dataFile
//                            << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(leg).p_[0] << "," :
//                    dataFile
//                            << 0.0 << ",";
//                    (final_trajectory.at(current_trajectory_point_index).ee_contact_.at(leg) == 1) ? dataFile
//                            << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(leg).p_[1] << "," :
//                    dataFile
//                            << -0.8 << ",";
//                    (final_trajectory.at(current_trajectory_point_index).ee_contact_.at(leg) == 1) ? dataFile
//                            << final_trajectory.at(current_trajectory_point_index).ee_motion_.at(leg).p_[2] << "," :
//                    dataFile
//                            << 1.6 << ",";
//                }
                dataFile << std::endl;
            }
            current_phase_start_index += horizons.at(i);
        }
        dataFile.close();
    }

void
TowrRosInterface::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = formulation_.terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = formulation_.terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}

} /* namespace towr */

