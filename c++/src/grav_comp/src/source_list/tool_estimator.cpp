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

#include <grav_comp/tool_estimator.h>
#include <cassert>

using namespace Eigen;

namespace arl
{

namespace tool_est_
{
  Eigen::Matrix<double, 3, 3> skewSymmetric(const Eigen::Vector3d &input)
  {
    Eigen::Matrix<double, 3, 3> result = Eigen::Matrix<double, 3, 3>::Zero();
    result(0, 1) = -input(2);
    result(0, 2) =  input(1);
    result(1, 0) =  input(2);
    result(1, 2) = -input(0);
    result(2, 0) = -input(1);
    result(2, 1) =  input(0);
    return result;
  }
}

ToolEstimator::ToolEstimator()
{
  gravity_vector_world = Eigen::Vector3d(0, 0, -GRAVITY_ACC);
}

ToolEstimator::ToolEstimator(const Vector3d& the_gravity_vector)
{
  gravity_vector_world = the_gravity_vector;
}

ToolEstimator::~ToolEstimator()
{
}

void ToolEstimator::estimatePayload(const std::vector<Vector6d>& wrench_data, const std::vector<Quaterniond>& orientation_data)
{
  assert(wrench_data.size() == orientation_data.size());

  unsigned int dataset_size = wrench_data.size();
  VectorXd force(3 * dataset_size), torque(3 * dataset_size), gravity_vector(3 * dataset_size);
  MatrixXd skew(3 * dataset_size, 3);

  for (unsigned int i = 0; i < dataset_size; i++)
  {
    force.segment(3 * i, 3) = wrench_data.at(i).segment(0, 3);
    torque.segment(3 * i, 3) = wrench_data.at(i).segment(3, 3);

    Vector3d gravity_vector_tool = orientation_data.at(i).toRotationMatrix().transpose() * this->gravity_vector_world;
    gravity_vector.segment(3 * i, 3) = gravity_vector_tool;

    skew.block(3 * i, 0, 3, 3) = tool_est_::skewSymmetric(gravity_vector_tool);
  }

  // Calculate mass with least squares for f = mg
  this->mass = (gravity_vector.transpose() * gravity_vector).inverse() * gravity_vector.transpose() * force;
  // this->mass = gravity_vector.bdcSvd(ComputeThinU | ComputeThinV).solve(force)[0];  // Use Eigen SVD

  // Calculate center of mass with least squares for torques = mass * center_of_mass cross_product gravity
  skew = - this->mass * skew;
  // Use normal equations for least squares:
  this->center_of_mass = (skew.transpose() * skew).inverse() * skew.transpose() * torque;
  // this->center_of_mass = skew.bdcSvd(ComputeThinU | ComputeThinV).solve(torque);  // This segfaults for some reason
}

Vector6d ToolEstimator::getGravityWrench(const Quaterniond& orientation)
{
  Vector6d wrench;
  Vector3d gravity = orientation.toRotationMatrix().transpose() * this->gravity_vector_world;
  Vector3d force = this->mass * gravity;
  Vector3d torque = this->mass * this->center_of_mass.cross(gravity);
  wrench << force, torque;
  return wrench;
}
}  // namespace arl
