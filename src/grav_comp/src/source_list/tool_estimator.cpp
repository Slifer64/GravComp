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
  this->mass = 0.0;
  this->CoM = Eigen::Vector3d::Zero();
  gravity_vector_world = Eigen::Vector3d(0, 0, -GRAVITY_ACC);
}

ToolEstimator::ToolEstimator(const Vector3d& the_gravity_vector)
{
  this->mass = 0.0;
  this->CoM = Eigen::Vector3d::Zero();
  gravity_vector_world = the_gravity_vector;
}

ToolEstimator::~ToolEstimator()
{
}

void ToolEstimator::estimatePayload(const std::vector<Vector6d>& wrench_data, const std::vector<Quaterniond>& quat_data)
{
  assert(wrench_data.size() == quat_data.size());

  unsigned n_data = wrench_data.size();
  VectorXd torque(3 * n_data), fz_world(n_data);
  MatrixXd minus_skew_force(3 * n_data, 3);

  int k = 0;
  for (unsigned int i = 0; i < n_data; i++)
  {
    torque.segment(k, 3) = wrench_data[i].segment(3, 3);

    Eigen::Vector3d f = wrench_data[i].segment(0, 3);
    Eigen::Vector3d f_world = quat_data[i].toRotationMatrix().transpose() * f;
    fz_world(i) = f_world(2);
    minus_skew_force.block(k, 0, 3, 3) = -tool_est_::skewSymmetric(f);

    k+=3;
  }

  // Estimate mass
  this->mass = fz_world.sum() / (n_data * this->gravity_vector_world(2));

  // Estimate center of mass with least squares for torques =  CoM x ( mass*gravity )
  // Use normal equations for least squares:
  this->CoM = (minus_skew_force.transpose() * minus_skew_force).inverse() * minus_skew_force.transpose() * torque;
  // this->CoM = skew.bdcSvd(ComputeThinU | ComputeThinV).solve(torque);  // This segfaults for some reason
}

// void ToolEstimator::estimatePayload(const std::vector<Vector6d>& wrench_data, const std::vector<Quaterniond>& quat_data)
// {
//   assert(wrench_data.size() == quat_data.size());
//
//   unsigned n_data = wrench_data.size();
//   VectorXd force(3 * n_data), torque(3 * n_data), gravity_vector(3 * n_data);
//   MatrixXd skew(3 * n_data, 3);
//
//   for (unsigned int i = 0; i < n_data; i++)
//   {
//     force.segment(3 * i, 3) = wrench_data.at(i).segment(0, 3);
//     torque.segment(3 * i, 3) = wrench_data.at(i).segment(3, 3);
//
//     Vector3d gravity_vector_tool = quat_data.at(i).toRotationMatrix().transpose() * this->gravity_vector_world;
//     gravity_vector.segment(3 * i, 3) = gravity_vector_tool;
//
//     skew.block(3 * i, 0, 3, 3) = tool_est_::skewSymmetric(gravity_vector_tool);
//   }
//
//   // Calculate mass with least squares for f = mg
//   this->mass = (gravity_vector.transpose() * gravity_vector).inverse() * gravity_vector.transpose() * force;
//
//   // Calculate center of mass with least squares for torques =  CoM x ( mass*gravity )
//   skew = - this->mass * skew;
//   // Use normal equations for least squares:
//   this->CoM = (skew.transpose() * skew).inverse() * skew.transpose() * torque;
//   // this->CoM = skew.bdcSvd(ComputeThinU | ComputeThinV).solve(torque);  // This segfaults for some reason
// }

Vector6d ToolEstimator::getGravityWrench(const Quaterniond& orientation) const
{
  Vector6d wrench;
  Vector3d force = orientation.toRotationMatrix().transpose() * (this->gravity_vector_world * this->mass);
  Vector3d torque = this->CoM.cross(force);
  wrench << force, torque;
  return wrench;
}
