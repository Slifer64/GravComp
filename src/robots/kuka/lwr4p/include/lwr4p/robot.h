#ifndef LWR4P_ROBOT_H
#define LWR4P_ROBOT_H

#include <FastResearchInterface.h>
#include <ros/ros.h>

#include <memory>
#include <vector>
#include <string>

#include <armadillo>

#include <lwr4p/robot_arm.h>

namespace as64_
{

namespace lwr4p_
{

class Robot : public RobotArm
{
public:
  Robot(const char *path_to_FRI_init=NULL);
  Robot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, const char *path_to_FRI_init=NULL);
  Robot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, const char *path_to_FRI_init=NULL);
  ~Robot();

  bool isOk() const override { return getMode()==lwr4p_::IDLE || const_cast<FastResearchInterface *>(FRI.get())->IsMachineOK(); }
  void enable() override;
  void setMode(const lwr4p_::Mode &m) override;
  void update() override;

  arma::vec getJointsPosition() const override
  {
    arma::vec output(N_JOINTS);
    static float temp[7];
    FRI->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < N_JOINTS; i++) {
      output(i) = temp[i];
    }
    return output;
  }

  arma::vec getJointsTorque() const override
  {
    arma::vec output(N_JOINTS);
    static float joint_torques[7];
    FRI->GetMeasuredJointTorques(joint_torques);
    output(0) = joint_torques[0];
    output(1) = joint_torques[1];
    output(2) = joint_torques[2];
    output(3) = joint_torques[3];
    output(4) = joint_torques[4];
    output(5) = joint_torques[5];
    output(6) = joint_torques[6];

    return output;
  }

  arma::vec getJointExternalTorque() const override
  {
    arma::vec output(N_JOINTS);
    static float estimated_external_joint_torques[7];
    FRI->GetEstimatedExternalJointTorques(estimated_external_joint_torques);
    output(0) = estimated_external_joint_torques[0];
    output(1) = estimated_external_joint_torques[1];
    output(2) = estimated_external_joint_torques[2];
    output(3) = estimated_external_joint_torques[3];
    output(4) = estimated_external_joint_torques[4];
    output(5) = estimated_external_joint_torques[5];
    output(6) = estimated_external_joint_torques[6];

    return output;
  }

  arma::mat getEEJacobian() const override
  {
    arma::mat output(6, N_JOINTS);

    FRI->GetCurrentJacobianMatrix(jacob_temp);
    std::vector<int> jac_indexes{0, 1, 2, 5, 4, 3};
    for (size_t i = 0; i < jac_indexes.size(); i++)
    {
      for (size_t j = 0; j < 7; j++)
      {
        output(i, j) = jacob_temp[jac_indexes[i]][j];
      }
    }

    return output;
  }

  arma::mat getJacobian() const override
  {
    arma::mat output = getEEJacobian();
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    arma::mat rot;
    rot << temp[0] << temp[1] << temp[2] << arma::endr
        << temp[4] << temp[5] << temp[6] << arma::endr
        << temp[8] << temp[9] << temp[10];
    output.submat(0, 0, 2, 6) = rot * output.submat(0, 0, 2, 6);
    output.submat(3, 0, 5, 6) = rot * output.submat(3, 0, 5, 6);

    return output;
  }

  arma::mat getTaskPose() const override
  {
    arma::mat output = arma::mat().eye(4, 4);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
        output(i, j) = temp[i * 4 + j];
      }
    }

    return output;
  }

  arma::vec getTaskPosition() const override
  {
    arma::vec output(3);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    output(0) = temp[3];
    output(1) = temp[7];
    output(2) = temp[11];

    return output;
  }

  arma::mat getTaskRotm() const override
  {
    arma::mat output(3,3);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    output(0, 0) = temp[0];
    output(0, 1) = temp[1];
    output(0, 2) = temp[2];
    output(1, 0) = temp[4];
    output(1, 1) = temp[5];
    output(1, 2) = temp[6];
    output(2, 0) = temp[8];
    output(2, 1) = temp[9];
    output(2, 2) = temp[10];

    return output;
  }

  arma::vec getTaskQuat() const override
  {
    return rotm2quat(getTaskRotm());
  }

  arma::vec getExternalWrench() const override
  {
   arma::vec output(6);
   static float estimated_external_cart_forces_and_torques[6];
   FRI->GetEstimatedExternalCartForcesAndTorques(estimated_external_cart_forces_and_torques);
   output(0) = estimated_external_cart_forces_and_torques[0];
   output(1) = estimated_external_cart_forces_and_torques[1];
   output(2) = estimated_external_cart_forces_and_torques[2];
   output(3) = estimated_external_cart_forces_and_torques[3];
   output(4) = estimated_external_cart_forces_and_torques[4];
   output(5) = estimated_external_cart_forces_and_torques[5];

   return output;
  }

  void setJointsPosition(const arma::vec &j_pos) override
  {
    if (getMode() != lwr4p_::Mode::JOINT_POS_CONTROL)
    {
      print_warn_msg("[lwr4+::Robot::setJointsPosition]: Cannot set joints position. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }

    if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    {
      printf("[JointPosController::setJointsPosition] Joint positions are commanded with closed controller.\n");
      printf("Opening controller ...\n");
      // start te cotroller
      startJointPositionController();
      // wait one tick
      FRI->WaitForKRCTick();
    }

    // temp variables
    static float temp[7];
    // put the values from arma to float[]
    for (int i = 0; i < 7; i++) temp[i] = j_pos(i);

    // set commanded joint positions to qd
    FRI->SetCommandedJointPositions(temp);
    // saveLastJointPosition(temp);
    last_joint_pos = j_pos;
  }

  void setJointsVelocity(const arma::vec &j_vel) override
  {
    if (getMode() != lwr4p_::Mode::JOINT_VEL_CONTROL)
    {
      print_warn_msg("[lwr4+::Robot::setJointsVelocity]: Cannot set joints velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }

    last_joint_pos += j_vel*getCtrlCycle();

    // temp variables
    static float temp[7];
    // put the values from arma to float[]
    for (int i = 0; i < 7; i++) {
      temp[i] = last_joint_pos(i);
    }
    FRI->SetCommandedJointPositions(temp);
  }

  void setJointsTorque(const arma::vec &j_torq) override
  {
    if (getMode() != lwr4p_::Mode::JOINT_TORQUE_CONTROL)
    {
      print_warn_msg("[lwr4+::Robot::setJointsTorque]: Cannot set joints torques. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }

    static float torques[7];
    for (size_t i = 0; i < 7; i++)
    {
      torques[i] = j_torq(i);
    }
    FRI->SetCommandedJointTorques(torques);
    // Mirror the joint positions and the cartesian pose in order to avoid
    // cartesian deviation errors
    float temp_position[7];
    FRI->GetMeasuredJointPositions(temp_position);
    FRI->SetCommandedJointPositions(temp_position);
    static float temp_pose[12];
    FRI->GetMeasuredCartPose(temp_pose);
    FRI->SetCommandedCartPose(temp_pose);
  }

  void setTaskVelocity(const arma::vec &task_vel) override
  {
    if (getMode() != lwr4p_::Mode::CART_VEL_CONTROL)
    {
      print_warn_msg("[lwr4+::Robot::setTaskVelocity]: Cannot set task velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }

    arma::vec j_vel = arma::solve(getJacobian(),task_vel);
    last_joint_pos += j_vel*getCtrlCycle();

    // temp variables
    static float temp[7];
    // put the values from arma to float[]
    for (int i = 0; i < 7; i++) {
      temp[i] = last_joint_pos(i);
    }
    FRI->SetCommandedJointPositions(temp);
  }

  void setTaskPose(const arma::mat &input) override
  {
    if (getMode() != lwr4p_::Mode::CART_IMPEDANCE_CONTROL)
    {
      print_warn_msg("[lwr4+::Robot::setTaskPose]: Cannot set task pose. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }

    if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    {
      printf("[CartImpedanceController::setTaskPose] Cartesian wrench is commanded with closed controller.\n");
      printf("Opening controller ...\n");
      // start te controller
      startCartImpController();
      // wait one tick
      FRI->WaitForKRCTick();
    }
    // temp variables
    static float temp[12];
    for (size_t i = 0; i < 3; i++)
      {
        for (size_t j = 0; j < 4; j++)
        {
          temp[i * 4 + j] = input(i, j);
        }
    }

    // set commanded task pose
    FRI->SetCommandedCartPose(temp);

  }

  void setWrench(const arma::vec &input) override
  {
    if (getMode() != lwr4p_::Mode::CART_IMPEDANCE_CONTROL)
    {
      print_warn_msg("[lwr4+::Robot::setWrench]: Cannot set wrench. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }

    if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    {
      printf("[CartImpedanceController::setWrench] Cartesian wrench is commanded with closed controller.\n");
      printf("Opening controller ...\n");
      // start te controller
      startCartImpController();
      // wait one tick
      FRI->WaitForKRCTick();
    }
    // temp variables
    static float temp[6];
    // put the values from arma to float[]
    for (int i = 0; i < 6; i++) {
      temp[i] = input(i);
    }

    // set commanded Cartesian forces/torques
    FRI->SetCommandedCartForcesAndTorques(temp);

    float temp_position[7];
    FRI->GetMeasuredJointPositions(temp_position);
    // FRI->SetCommandedJointPositions(temp_position);
    // float temp_pose[12];
    // FRI->GetMeasuredCartPose(temp_pose);
    // FRI->SetCommandedCartPose(temp_pose);
    // saveLastJointPosition(temp_position);
  }

  void setCartStiffness(const arma::vec &cart_stiff) override;

  void setCartDamping(const arma::vec &cart_damp) override;

private:

  void setZeroJointTorques()
  {
    static float torques[7];
    for (size_t i = 0; i < 7; i++)
    {
      torques[i] = 0;
    }
    FRI->SetCommandedJointTorques(torques);
    // Mirror the joint positions and the cartesian pose in order to avoid
    // cartesian deviation errors
    float temp_position[7];
    FRI->GetMeasuredJointPositions(temp_position);
    FRI->SetCommandedJointPositions(temp_position);
    static float temp_pose[12];
    FRI->GetMeasuredCartPose(temp_pose);
    FRI->SetCommandedCartPose(temp_pose);
  }

  std::shared_ptr<FastResearchInterface> FRI;
  void startJointPositionController();
  void startJointTorqueController();
  void startCartImpController();
  void stop() override;
  void protectiveStop() override;

  void initRobot(const char *path_to_FRI_init = NULL);

  mutable float **jacob_temp;

  arma::vec last_joint_pos;
};

}  // namespace lwr4p_

} // namespace as64_

#endif  // LWR4P_ROBOT_H
