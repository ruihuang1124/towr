/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/hyq/hyq_spliner.h>
#include <kindr/rotations/RotationEigen.hpp>

namespace xpp {
namespace hyq {

using namespace xpp::utils;

void HyqSpliner::SetParams(double upswing,
               double lift_height,
               double outward_swing_distance)
{
  kUpswingPercent = upswing;
  kLiftHeight = lift_height;
  kOutwardSwingDistance = outward_swing_distance;

}

void HyqSpliner::Init(const HyqState& P_init,
                      const xpp::zmp::PhaseVec& phase_info,
                      const VecPolyomials& optimized_xy_spline,
                      const VecFoothold& footholds,
                      double robot_height)
{
  nodes_ = BuildPhaseSequence(P_init, phase_info, optimized_xy_spline, footholds, robot_height);
  CreateAllSplines(nodes_);
  optimized_xy_spline_ = optimized_xy_spline;
}

void
HyqSpliner::Init (const xpp::zmp::PhaseVec& phase_info, const VecPolyomials& com_spline,
                  const VecFoothold& contacts, double robot_height)
{
  // cmo this should not need to be explicitly defined, merge both Init functions
  HyqState x0;
  x0.ZeroVelAcc();
  x0.base_.lin.p.topRows<kDim2d>() = xpp::zmp::ComPolynomial::GetCOM(0.0, com_spline).p;
  x0.base_.lin.v.topRows<kDim2d>() = xpp::zmp::ComPolynomial::GetCOM(0.0, com_spline).v;
  x0.base_.lin.a.topRows<kDim2d>() = xpp::zmp::ComPolynomial::GetCOM(0.0, com_spline).a;
  x0.base_.lin.p.z() = robot_height;

  for (auto f : phase_info.at(0).fixed_contacts_) {
    x0.feet_[f.leg].p = f.p;
    x0.swingleg_[f.leg] = false;
  }

  Init(x0,phase_info,com_spline,contacts,robot_height);
}


HyqSpliner::RobotStateTraj
HyqSpliner::BuildWholeBodyTrajectory () const
{
  RobotStateTraj trajectory;

  double controller_loop_interval = 0.004; // 250Hz SL task servo

  double t=0.0;
  while (t<GetTotalTime()) {

    LegDataMap<Point> feet;
    LegDataMap<bool> swingleg;

    auto pos  = GetCurrPosition(t);
    auto ori  = GetCurrOrientation(t);
    FillCurrFeet(t, feet, swingleg);

    HyqStateStamped state;
    state.base_.lin = pos;
    state.base_.ang = ori;
    state.feet_ = feet;
    state.swingleg_ = swingleg;
    state.t_ = t;

    trajectory.push_back(state);

    t += controller_loop_interval;
  }

  return trajectory;
}



std::vector<SplineNode>
HyqSpliner::BuildPhaseSequence(const HyqState& P_init,
                               const xpp::zmp::PhaseVec& phase_info,
                               const VecPolyomials& optimized_xy_spline,
                               const VecFoothold& footholds,
                               double robot_height)
{
  std::vector<SplineNode> nodes;

  // start node
  SplineNode init_node = BuildNode(P_init, 0.0);
  nodes.push_back(init_node); // this node has to be reached instantly (t=0)

  /** Add state sequence based on footsteps **/
  // desired state after first 4 leg support phase
  HyqState P_plan_prev = P_init;
  P_plan_prev.ZeroVelAcc(); // these aren't used anyway, overwritten by optimizer
  for (hyq::LegID l : hyq::LegIDArray) {
    P_plan_prev.feet_[l].p(Z) = 0.0;
  }
  P_plan_prev.base_.lin.p(Z) = robot_height + P_plan_prev.GetZAvg(); // height of footholds

  double t_global = 0.0;
  for (const auto& curr_phase : phase_info)
  {
    // copy a few values from previous state
    HyqState goal_node = P_plan_prev;

    // current contact configuration during the phase
    VecFoothold contacts;
    for (auto c : curr_phase.free_contacts_) {
      goal_node.feet_[static_cast<LegID>(c.ee)].p   = footholds.at(c.id).p;
      goal_node.swingleg_[static_cast<LegID>(c.ee)] = false;
    }

    for (auto f : curr_phase.fixed_contacts_) {
      goal_node.feet_[f.leg].p = f.p;
      goal_node.swingleg_[f.leg] = false;
    }

    // add the desired foothold after swinging if this is a step phase
    if (curr_phase.IsStep()) {
      int step = curr_phase.n_completed_steps_;
      LegID step_leg = footholds.at(step).leg;
      auto goal_of_step = footholds.at(step);
      goal_node.feet_[step_leg].p   = footholds.at(step).p;
      goal_node.swingleg_[step_leg] = true; // currently this leg is swinging
    }

    // adjust orientation depending on footholds
    std::array<Vector3d, kNumSides> avg = goal_node.GetAvgSides();
    // fixme calculate distance based on current foothold position, not fixed values
    double width_hip = 0.414;
    double length_hip = 0.747;
    double i_roll  = std::atan2((avg[LEFT_SIDE](Z) - avg[RIGHT_SIDE](Z)), width_hip);
    double i_pitch = std::atan2((avg[HIND_SIDE](Z) - avg[FRONT_SIDE](Z)), length_hip);


    // how strictly the body should follow difference in foothold height
    double roll_gain = 0.0;
    double pitch_gain = 0.0;
    // Quaternion is the same for angle += i*pi
    kindr::rotations::eigen_impl::EulerAnglesXyzPD yprIB(
        roll_gain*i_roll,
        pitch_gain*i_pitch,
        0.0); // P_yaw_des.at(s.step_));


    kindr::rotations::eigen_impl::RotationQuaternionPD qIB(yprIB);
    goal_node.base_.ang.q = qIB.toImplementation();

    // adjust global z position of body depending on footholds
    goal_node.base_.lin.p(Z) = robot_height + goal_node.GetZAvg(); // height of footholds

    // fill in x-y position, although overwritten anyway later
    double t_phase = curr_phase.duration_;
    t_global += t_phase;
    auto com_xy_at_end_of_phase = xpp::zmp::ComPolynomial::GetCOM(t_global, optimized_xy_spline);
    goal_node.base_.lin.p.topRows(kDim2d) = com_xy_at_end_of_phase.p;

    nodes.push_back(BuildNode(goal_node, t_phase));

    P_plan_prev = goal_node;
  }

  return nodes;
}

void HyqSpliner::CreateAllSplines(const std::vector<SplineNode>& nodes)
{
  pos_spliner_.clear();
  ori_spliner_.clear();
  feet_spliner_up_.clear();
  feet_spliner_down_.clear();

  Spliner3d pos, ori;
  LegDataMap< Spliner3d > feet_up, feet_down;
  std::cout << "nodes.size(): " << nodes.size() << std::endl;
  for (int n=1; n<nodes_.size(); ++n) {
    SplineNode from = nodes.at(n-1);
    SplineNode to   = nodes.at(n);

    BuildOneSegment(from, to, pos, ori, feet_up, feet_down);
    pos_spliner_.push_back(pos);
    ori_spliner_.push_back(ori);
    feet_spliner_up_.push_back(feet_up);
    feet_spliner_down_.push_back(feet_down);
  }
}


// fixme use quaternion in state directly, don't map to roll-pitch-yaw
SplineNode
HyqSpliner::BuildNode(const HyqState& state, double t_max)
{
  Point ori_rpy(TransformQuatToRpy(state.base_.ang.q));
  return SplineNode(state, ori_rpy, t_max);
}


Eigen::Vector3d
HyqSpliner::TransformQuatToRpy(const Eigen::Quaterniond& q)
{
  // wrap orientation
  kindr::rotations::eigen_impl::RotationQuaternionPD qIB(q);

  kindr::rotations::eigen_impl::EulerAnglesXyzPD rpyIB(qIB);
  rpyIB.setUnique(); // wrap euler angles yaw from -pi..pi

  // if yaw jumped over range from -pi..pi
  static double yaw_prev = 0.0;
  static int counter360 = 0;
  if (rpyIB.yaw()-yaw_prev < -M_PI_2) {
    std::cout << "passed yaw=0.9pi->-0.9pi, increasing counter...\n";
    counter360 += 1;
  }
  if (rpyIB.yaw()-yaw_prev > M_PI_2) {
    std::cout << "passed yaw=-0.9pi->0.9pi, decreasing counter...\n";
    counter360 -= 1;
  }
  yaw_prev = rpyIB.yaw();

  // contains information that orientation went 360deg around
  kindr::rotations::eigen_impl::EulerAnglesXyzPD yprIB_full = rpyIB;
  yprIB_full.setYaw(rpyIB.yaw() + counter360*2*M_PI);

  return yprIB_full.toImplementation();
}


double HyqSpliner::GetTotalTime() const
{
  double T = 0.0;
  for (uint n = 1; n < nodes_.size(); n++) {
    T += nodes_.at(n).T;
  }
  return T;
}


double HyqSpliner::GetLocalSplineTime(double t_global) const
{
  int spline = GetSplineID(t_global);
  int goal_node = spline+1;

  double t_local = t_global;
  for (int n = 1; n < goal_node; n++) {
    t_local -= nodes_.at(n).T;
  }
  return t_local;
}


int HyqSpliner::GetSplineID(double t_global) const
{
  assert(t_global <= GetTotalTime()); // time inside the time frame

  double t = 0;
  for (uint n=1; n<nodes_.size(); ++n) {
    t += nodes_.at(n).T;

    if (t >= t_global - 1e-4) // so at "equal", previous spline is returned
      return n-1; // since first spline connects node 0 and 1
  }
  assert(false); // this should never be reached
}


SplineNode HyqSpliner::GetGoalNode(double t_global) const
{
  int spline = GetSplineID(t_global);
  int goal_node = spline+1;
  return nodes_.at(goal_node);
}



Eigen::Vector3d
HyqSpliner::GetCurrZState(double t_global) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

  Point pos;
  pos_spliner_.at(spline).GetPoint(t_local, pos);

  Eigen::Vector3d z_state;

  z_state(xpp::utils::kPos) = pos.p.z();
  z_state(xpp::utils::kVel) = pos.v.z();
  z_state(xpp::utils::kAcc) = pos.a.z();

  return z_state;
}


HyqSpliner::Point
HyqSpliner::GetCurrPosition(double t_global) const
{
  Vector3d z_splined = GetCurrZState(t_global);
  xpp::utils::BaseLin2d xy_optimized = xpp::zmp::ComPolynomial::GetCOM(t_global, optimized_xy_spline_);

  Point pos;

  pos.p.segment(0,2) = xy_optimized.p;// - (P_R_Bdes*b_r_geomtocog).segment<2>(X);
  pos.p.z()          = z_splined(xpp::utils::kPos);
  pos.v.segment(0,2) = xy_optimized.v;
  pos.v.z()          = z_splined(xpp::utils::kVel);
  pos.a.segment(0,2) = xy_optimized.a;
  pos.a.z()          = z_splined(xpp::utils::kAcc);

  return pos;
}


xpp::utils::BaseAng3d
HyqSpliner::GetCurrOrientation(double t_global) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

  Point ori_rpy;
  ori_spliner_.at(spline).GetPoint(t_local, ori_rpy);

  // transform to orientation with quaternion
  xpp::utils::BaseAng3d ori;
  kindr::rotations::eigen_impl::EulerAnglesXyzPD yprIB(ori_rpy.p);
  kindr::rotations::eigen_impl::RotationQuaternionPD qIB(yprIB);
  ori.q = qIB.toImplementation();
  ori.v = ori_rpy.v;
  ori.a = ori_rpy.a;

  return ori;
}


void
HyqSpliner::FillCurrFeet(double t_global,
                        LegDataMap<Point>& feet,
                        LegDataMap<bool>& swingleg) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);
  int  goal_node = spline+1;

  swingleg = false;

  feet = nodes_[goal_node].state_.feet_;

  int sl = nodes_[goal_node].state_.SwinglegID();
  if (sl != NO_SWING_LEG) // only spline foot of swingleg
  {
    double t_upswing = nodes_[goal_node].T * kUpswingPercent;

    if ( t_local < t_upswing) // leg swinging up
      feet_spliner_up_.at(spline)[sl].GetPoint(t_local, feet[sl]);
    else // leg swinging down
      feet_spliner_down_.at(spline)[sl].GetPoint(t_local - t_upswing, feet[sl]);

    swingleg[sl] = true;
  }
}

void HyqSpliner::BuildOneSegment(const SplineNode& from, const SplineNode& to,
                                 Spliner3d& pos, Spliner3d& ori,
                                 LegDataMap< Spliner3d >& feet_up,
                                 LegDataMap< Spliner3d >& feet_down) const
{
  pos = BuildPositionSpline(from, to);
  ori = BuildOrientationRpySpline(from, to);
  feet_up = BuildFootstepSplineUp(from, to);

  // this is the outter/upper-most point the foot swings to
  LegDataMap<Point> f_switch;
  double t_switch = to.T * kUpswingPercent;
  for (LegID leg : LegIDArray) {
     feet_up[leg].GetPoint(t_switch, f_switch[leg]);
  }

  feet_down = BuildFootstepSplineDown(f_switch, to);
}


HyqSpliner::Spliner3d
HyqSpliner::BuildPositionSpline(const SplineNode& from, const SplineNode& to) const
{
  Spliner3d pos;
  pos.SetBoundary(to.T, from.state_.base_.lin, to.state_.base_.lin);
  return pos;
}


HyqSpliner::Spliner3d
HyqSpliner::BuildOrientationRpySpline(const SplineNode& from, const SplineNode& to) const
{
  Spliner3d ori;
  ori.SetBoundary(to.T, from.ori_rpy_, to.ori_rpy_);
  return ori;
}


xpp::hyq::LegDataMap<HyqSpliner::Spliner3d>
HyqSpliner::BuildFootstepSplineUp(const SplineNode& from, const SplineNode& to) const
{
  LegDataMap< Spliner3d > feet_up;

  // Feet spliner for all legs, even if might be stance legs
  for (LegID leg : LegIDArray) {

    // raise intermediate foothold dependant on foothold difference
    double delta_z = std::abs(to.state_.feet_[leg].p(Z) - from.state_.feet_[leg].p(Z));
    Point foot_raised = to.state_.feet_[leg];
    foot_raised.p[Z] += kLiftHeight + delta_z;

    // move outward only if footholds significantly differ in height
    if (delta_z > 0.01) {
      int sign = (leg == LH || leg == LF) ? 1 : -1;
      foot_raised.p[Y] += sign * kOutwardSwingDistance;
    }

    // upward swing
    double swing_time = to.T;
    feet_up[leg].SetBoundary(swing_time, from.state_.feet_[leg], foot_raised);
  }

  return feet_up;
}


xpp::hyq::LegDataMap<HyqSpliner::Spliner3d>
HyqSpliner::BuildFootstepSplineDown(const LegDataMap<Point>& feet_at_switch, const SplineNode& to) const
{
  LegDataMap< Spliner3d > feet_down;

  // Feet spliner for all legs, even if might be stance legs
  for (LegID leg : LegIDArray) {

    // downward swing from the foothold at switch state to original
    double duration = to.T * (1.0-kUpswingPercent);
    feet_down[leg].SetBoundary(duration, feet_at_switch[leg], to.state_.feet_[leg]);
  }

  return feet_down;
}


} // namespace hyq
} // namespace xpp
