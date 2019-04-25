#ifndef AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H
#define AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H

#include <Eigen/Dense>
#include <mutex>

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
  explicit ToolEstimator(const Eigen::Vector3d &the_gravity_vector);

  ~ToolEstimator();

  void setCoM(const Eigen::Vector3d &CoM) { std::unique_lock<std::mutex>(mass_mtx); this->CoM = CoM; }
  void setMass(double mass) { std::unique_lock<std::mutex>(mass_mtx); this->mass = mass; }

  Eigen::Vector3d getCoM() const  { std::unique_lock<std::mutex>(mass_mtx); return this->CoM; }
  double getMass() const  { std::unique_lock<std::mutex>(mass_mtx); return this->mass; }

  /**
   * \brief Estimates the payload of the tool (mass, center of mass), given
   * the recorded wrenches at the tool expressed in the tool frame and the orientation
   * of the tool w.r.t. the base frame.
   *
   * @param[in] wrench_data Recorded wrench expressed in the tool frame.
   * @param[in] quat_data The orientation of the tool w.r.t. the base frame.
   * wrench
   */
  void estimatePayload(const std::vector<Eigen::Vector6d>& wrench_data, const std::vector<Eigen::Quaterniond>& quat_data);

  /**
   * \brief Calculates the wrench of the tool expressed in the tool frame.
   * @param[in] orientation Orientation as unit quaternion, of the tool frame w.r.t the base frame.
   * @return The wrench of the tool expressed in the tool frame.
   */
  Eigen::Vector6d getToolWrench(const Eigen::Quaterniond& orientation) const;

private:

  Eigen::Vector3d gravity_vector_world;  ///< The gravity vector w.r.t. the world frame

  std::mutex mass_mtx;
  double mass;  ///< The mass of the tool

  std::mutex CoM_mtx;
  Eigen::Vector3d CoM;  ///< The center of mass of the tool
};

#endif  // AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H
