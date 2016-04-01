/*
 * constraints.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {


class Constraints {

public:
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Footholds;
  typedef xpp::utils::MatVec MatVec;

  struct Bound {
    Bound(double lower, double upper) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;
  };

public:
  Constraints (const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
               const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
               const MatVec& qp_equality_constraints);
  virtual
  ~Constraints () {};

  Eigen::VectorXd EvalContraints(const Footholds& footholds,
                                 const Eigen::VectorXd& x_coeff);

  xpp::hyq::SupportPolygonContainer supp_polygon_container_;
  xpp::zmp::ContinuousSplineContainer zmp_spline_container_;

  MatVec spline_junction_constraints_;

  Eigen::VectorXd g_;
  std::vector<Constraints::Bound> bounds_;
  const std::vector<xpp::hyq::Foothold> planned_footholds_;

private:

  MatVec x_zmp_;
  MatVec y_zmp_;
  bool first_constraint_eval_ = true;

  Eigen::VectorXd EvalSuppPolygonConstraints(const Footholds& footholds,
                                             const Eigen::VectorXd& x_coeff);

  Eigen::VectorXd EvalFootholdConstraints(const Footholds& footholds);

  Eigen::VectorXd EvalSplineJunctionConstraints(const Eigen::VectorXd& x_coeff);

  Eigen::VectorXd EvalStepLengthConstraints(const Footholds& footholds);


  void AddBounds(int m_constraints, double lower, double upper);
  /**
   * Changes zmp_spline_container
   * @param x_coeff
   * @param footholds
   * @param bounds
   * @return
   */
  Eigen::VectorXd EvalWorkspaceConstraints(const Eigen::VectorXd& x_coeff,
                                           const Footholds& footholds);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_ */
