#ifndef AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H
#define AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H

#include <Eigen/Dense>

namespace Eigen
{
typedef Matrix<double,6,1> Vector6d;
}

const double GRAVITY_ACC = 9.80665;

/*!
 * \brief Estimates the payload of a tool based on measurements and calculates
 * the gravity wrench of the tool.
 */
class ToolEstimator
{
public:
  ToolEstimator();
  explicit ToolEstimator(const Eigen::Vector3d& the_gravity_vector);

  ~ToolEstimator();

  /**
   * \brief Estimates the payload of the tool (mass, center of mass), given
   * measurement data.
   *
   * \param wrench_data A number of recorded wrenches
   * \param quat_data The orientation w.r.t. the world for each recorded
   * wrench
   */
  void estimatePayload(const std::vector<Eigen::Vector6d>& wrench_data, const std::vector<Eigen::Quaterniond>& quat_data);

  /**
   * \brief Calculates the gravity wrench of the tool based on the orientation
   */
  Eigen::Vector6d getGravityWrench(const Eigen::Quaterniond& orientation) const;

  Eigen::Vector3d gravity_vector_world;  ///< The gravity vector w.r.t. the world frame
  double mass;  ///< The mass of the tool
  Eigen::Vector3d CoM;  ///< The center of mass of the tool
};

#endif  // AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H
