/*******************************************************************************
 * Copyright (c) 2016-2019 Automation and Robotics Lab, AUTh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#ifndef AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H
#define AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H

#include <Eigen/Dense>
// #include <autharl_core/eigen_plugins.h>

namespace Eigen
{
typedef Matrix<double,6,1> Vector6d;
}

const double GRAVITY_ACC = 9.80665;

namespace arl
{
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
   * \param orientation_data The orientation w.r.t. the world for each recorded
   * wrench
   */
  void estimatePayload(const std::vector<Eigen::Vector6d>& wrench_data, const std::vector<Eigen::Quaterniond>& orientation_data);

  /**
   * \brief Calculates the gravity wrench of the tool based on the orientation
   */
  Eigen::Vector6d getGravityWrench(const Eigen::Quaterniond& orientation);

  Eigen::Vector3d gravity_vector_world;  /**< The gravity vector w.r.t. the world frame */
  double mass;  /**< The mass of the tool */
  Eigen::Vector3d center_of_mass;  /**< The center of mass of the tool */
};
}  // namespace arl
#endif  // AUTHARL_CORE_MATH_TOOL_ESTIMATOR_H
