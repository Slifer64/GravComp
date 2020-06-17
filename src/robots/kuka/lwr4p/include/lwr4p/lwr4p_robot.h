#ifndef LWR_LWRp_ROBOT_H
#define LWR_LWRp_ROBOT_H

#include <FastResearchInterface.h>

#include <lwr4p/robot.h>

namespace lwr4p
{

class LWR4pRobot : public Robot
{
public:
  LWR4pRobot(const char *path_to_FRI_init=NULL);
  ~LWR4pRobot();
  void stop() { FRI->StopRobot(); }
  bool isOk() const { return const_cast<FastResearchInterface *>(FRI.get())->IsMachineOK(); }
  void setMode(lwr4p::Mode mode);
  void waitNextCycle() { FRI->WaitForKRCTick(); }
  double getControlCycle() const { return cycle; }

  arma::vec getJointPosition() const
  {
    arma::vec output(N_JOINTS);
    static float temp[7];
    const_cast<FastResearchInterface *>(FRI.get())->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < N_JOINTS; i++) {
      output(i) = temp[i];
    }
    return output;
  }

  arma::vec getJointTorque() const
  {
    arma::vec output(N_JOINTS);
    static float joint_torques[7];
    const_cast<FastResearchInterface *>(FRI.get())->GetMeasuredJointTorques(joint_torques);
    output(0) = joint_torques[0];
    output(1) = joint_torques[1];
    output(2) = joint_torques[2];
    output(3) = joint_torques[3];
    output(4) = joint_torques[4];
    output(5) = joint_torques[5];
    output(6) = joint_torques[6];

    return output;
  }

  arma::vec getJointExternalTorque() const
  {
    arma::vec output(N_JOINTS);
    static float estimated_external_joint_torques[7];
    const_cast<FastResearchInterface *>(FRI.get())->GetEstimatedExternalJointTorques(estimated_external_joint_torques);
    output(0) = estimated_external_joint_torques[0];
    output(1) = estimated_external_joint_torques[1];
    output(2) = estimated_external_joint_torques[2];
    output(3) = estimated_external_joint_torques[3];
    output(4) = estimated_external_joint_torques[4];
    output(5) = estimated_external_joint_torques[5];
    output(6) = estimated_external_joint_torques[6];

    return output;
  }

  arma::mat getEEJacobian() const
  {
    arma::mat output(6, N_JOINTS);

    const_cast<FastResearchInterface *>(FRI.get())->GetCurrentJacobianMatrix(jacob_temp);
    std::vector<int> jac_indexes{0, 1, 2, 5, 4, 3};
    float **jacob = const_cast<float **>(jacob_temp);
    for (size_t i = 0; i < jac_indexes.size(); i++)
    {
      for (size_t j = 0; j < 7; j++)
      {
        output(i, j) = jacob[jac_indexes[i]][j];
      }
    }

    return output;
  }

  arma::mat getRobotJacobian() const
  {
    arma::mat output = getEEJacobian();
    static float temp[12];
    const_cast<FastResearchInterface *>(FRI.get())->GetMeasuredCartPose(temp);
    arma::mat rot;
    rot << temp[0] << temp[1] << temp[2] << arma::endr
        << temp[4] << temp[5] << temp[6] << arma::endr
        << temp[8] << temp[9] << temp[10];
    output.submat(0, 0, 2, 6) = rot * output.submat(0, 0, 2, 6);
    output.submat(3, 0, 5, 6) = rot * output.submat(3, 0, 5, 6);

    return output;
  }

  arma::mat getTaskPose() const
  {
    arma::mat output = arma::mat().eye(4, 4);
    static float temp[12];
    const_cast<FastResearchInterface *>(FRI.get())->GetMeasuredCartPose(temp);
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
        output(i, j) = temp[i * 4 + j];
      }
    }

    return output;
  }

  arma::vec getTaskPosition() const
  {
    arma::vec output(3);
    static float temp[12];
    const_cast<FastResearchInterface *>(FRI.get())->GetMeasuredCartPose(temp);
    output(0) = temp[3];
    output(1) = temp[N_JOINTS];
    output(2) = temp[11];

    return output;
  }

  arma::mat getTaskOrientation() const
  {
    arma::mat output(3,3);
    static float temp[12];
    const_cast<FastResearchInterface *>(FRI.get())->GetMeasuredCartPose(temp);
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

  arma::vec getExternalWrench() const
  {
    arma::vec output(6);
    static float estimated_external_cart_forces_and_torques[6];
    const_cast<FastResearchInterface *>(FRI.get())->GetEstimatedExternalCartForcesAndTorques(estimated_external_cart_forces_and_torques);
    output(0) = estimated_external_cart_forces_and_torques[0];
    output(1) = estimated_external_cart_forces_and_torques[1];
    output(2) = estimated_external_cart_forces_and_torques[2];
    output(3) = estimated_external_cart_forces_and_torques[3];
    output(4) = estimated_external_cart_forces_and_torques[4];
    output(5) = estimated_external_cart_forces_and_torques[5];

    return output;
  }

  void setJointPosition(const arma::vec &jpos)
  {
    if (this->mode == lwr4p::Mode::JOINT_POS_CTRL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[JointPosController::setJointPosition] Joint positions are commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te cotroller
        startJointPositionController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      static float temp[7];
      // put the values from arma to float[]
      for (int i = 0; i < 7; i++) {
        temp[i] = jpos(i);
      }
      // set commanded joint positions to qd
      FRI->SetCommandedJointPositions(temp);
      saveLastJointPosition(temp);
    }
    else PRINT_WARNING_MSG("[LWR4pRobot::setJointPosition]: setJointPosition only available in JOINT_POS_CTRL mode.\n");

  }

  void setJointVelocity(const arma::vec &jvel)
  {
    if (this->mode == lwr4p::Mode::JOINT_VEL_CTRL)
    {
      static float temp[7];
      for (size_t i = 0; i < 7; i++)
      {
        last_jnt_pos[i] += jvel(i) * cycle;
      }
      FRI->SetCommandedJointPositions(last_jnt_pos);
    }
    else PRINT_WARNING_MSG("[LWR4pRobot::setJointVelocity]: setJointVelocity only available in JOINT_VEL_CTRL mode.\n");
  }

  void setJointTorque(const arma::vec &jtorq)
  {
    if (this->mode == lwr4p::Mode::JOINT_TORQUE_CTRL)
    {
      static float torques[7];
      for (size_t i = 0; i < 7; i++)
      {
        torques[i] = jtorq(i);
      }
      FRI->SetCommandedJointTorques(torques);
      // Mirror the joint positions and the cartesian pose in order to avoid
      // cartesian deviation errors
      float temp_position[N_JOINTS];
      FRI->GetMeasuredJointPositions(temp_position);
      FRI->SetCommandedJointPositions(temp_position);
      static float temp_pose[12];
      FRI->GetMeasuredCartPose(temp_pose);
      FRI->SetCommandedCartPose(temp_pose);

      saveLastJointPosition(temp_position);
    }
    else PRINT_WARNING_MSG("[LWR4pRobot::setJointTorque]: setJointTorque only available in JOINT_TORQUE_CTRL mode.\n");

  }

  void setTaskPose(const arma::mat &task_pose)
  {
    if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
    {
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
            temp[i * 4 + j] = task_pose(i, j);
          }
      }

      // set commanded task pose
      FRI->SetCommandedCartPose(temp);
    }
    else PRINT_WARNING_MSG("[LWR4pRobot::setTaskPose]: setTaskPose only available in IMPEDANCE_CONTROL mode.\n");

  }

  void setWrench(const arma::vec &wrench)
  {
    if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
    {
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
        temp[i] = wrench(i);
      }

      // set commanded Cartesian forces/torques
      FRI->SetCommandedCartForcesAndTorques(temp);

      float temp_position[N_JOINTS];
      FRI->GetMeasuredJointPositions(temp_position);
      // FRI->SetCommandedJointPositions(temp_position);
      // float temp_pose[12];
      // FRI->GetMeasuredCartPose(temp_pose);
      // FRI->SetCommandedCartPose(temp_pose);
      saveLastJointPosition(temp_position); //needed for numeric differentation to obtain q_dot [isn't it?]

    }
    else PRINT_WARNING_MSG("[LWR4pRobot::setWrench]: setWrench only available in IMPEDANCE_CONTROL mode.\n");
  }

  void setJointTrajectory(const arma::vec &jtarget, double duration);

  void setCartStiffness(const arma::vec &cart_stiff);

  void setCartDamping(const arma::vec &cart_damp);

private:
  std::shared_ptr<FastResearchInterface> FRI;
  void startJointPositionController();
  void startJointTorqueController();
  void startCartImpController();
  void stopController();
  void saveLastJointPosition(float input[7]);
  void saveLastJointPosition();
  float last_jnt_pos[7];


  float **jacob_temp;

};


}  // namespace lwr4p

#endif  // LWR_LWRp_ROBOT_H
