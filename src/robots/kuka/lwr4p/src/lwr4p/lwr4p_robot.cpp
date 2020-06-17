/**
 * Copyright (C) 2016 AUTH-lwr
 */

#include <lwr4p/lwr4p_robot.h>
#include <iostream>
#include <string>

#include <ros/package.h>

namespace lwr4p
{

  LWR4pRobot::LWR4pRobot(const char *path_to_FRI_init)
  {
    if (path_to_FRI_init == NULL)
    {
      std::string path = ros::package::getPath("lwr4p") + "/FRILibrary/980500-FRI-Driver.init";
      FRI.reset(new FastResearchInterface(path.c_str()));
    }
    else FRI.reset(new FastResearchInterface(path_to_FRI_init));

    cycle = FRI->GetFRICycleTime();
    this->mode = lwr4p::Mode::UNDEFINED;  // Dummy initialization before stopping controller
    stopController();  // Initially the robot is stopped
    startJointPositionController();
    waitNextCycle();

    // preallocate memory for jacobian
    jacob_temp = reinterpret_cast<float**>(malloc(sizeof(float *) * 6));
    for (size_t i = 0; i < 6; i++) {
      jacob_temp[i] = reinterpret_cast<float*>(malloc(sizeof(float) * 7));
    }
  }

  LWR4pRobot::~LWR4pRobot()
  {
    for (size_t i = 0; i < 6; i++) free(jacob_temp[i]);
    free(jacob_temp);
  }

  void LWR4pRobot::setMode(lwr4p::Mode mode)
  {
    if (this->mode == mode) return;

    switch (mode)
    {
      case lwr4p::Mode::STOPPED:
        stopController();
        saveLastJointPosition();
        this->mode = lwr4p::Mode::STOPPED;
        break;
      case lwr4p::Mode::JOINT_POS_CTRL:
        if (this->mode != lwr4p::Mode::JOINT_VEL_CTRL)
        {
          stopController();
          startJointPositionController();
        }
        this->mode = lwr4p::Mode::JOINT_POS_CTRL;
        saveLastJointPosition();
        break;
      case lwr4p::Mode::JOINT_VEL_CTRL:
        if (this->mode != lwr4p::Mode::JOINT_POS_CTRL)
        {
          stopController();
          startJointPositionController();
        }
        this->mode = lwr4p::Mode::JOINT_VEL_CTRL;
        saveLastJointPosition();
        break;
      case lwr4p::Mode::JOINT_TORQUE_CTRL:
        stopController();
        startJointTorqueController();
        saveLastJointPosition();
        break;
      case lwr4p::Mode::IMPEDANCE_CONTROL:
        stopController();
        startCartImpController();
        saveLastJointPosition();
        break;
      default:
        throw std::runtime_error("[LWR4pRobot::setMode]: Invalid mode " + getModeName(mode) + "...");
    }
  }

  void LWR4pRobot::setJointTrajectory(const arma::vec &input, double duration)
  {
    // setJntPosTrajTemplate(input, duration)
    // inital joint position values
    arma::vec q0 = arma::zeros<arma::vec>(N_JOINTS);
    arma::vec temp = arma::zeros<arma::vec>(N_JOINTS);
    for (int i = 0; i < N_JOINTS; i++) {
      temp(i) = input(i);
    }
    q0 = getJointPosition();
    // keep last known robot mode
    lwr4p::Mode prev_mode = mode;
    arma::vec qref = q0;
    // start conttroller
    setMode(lwr4p::Mode::JOINT_POS_CTRL);
    // initalize time
    double t = 0.0;
    // the main while
    while (t < duration)
    {
      // waits for the next tick also
      FRI->WaitForKRCTick();
      // compute time now
      t += cycle;
      // update trajectory
      qref = get5thOrder(t, q0, temp, duration).col(0);
      // set joint positions
      setJointPosition(qref);
    }
    // reset last known robot mode
    setMode(prev_mode);
  }

  void LWR4pRobot::setCartStiffness(const arma::vec &input)
  {
    if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setCartStiffness] Cartesian wrench is commanded with closed controller.\n");
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

      // set value
      FRI->SetCommandedCartStiffness(temp);
    }
    else
    {
      std::cerr << "setCartStiffness only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void LWR4pRobot::setCartDamping(const arma::vec &input)
  {
    if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setCartDamping] Cartesian wrench is commanded with closed controller.\n");
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

      // set value
      FRI->SetCommandedCartDamping(temp);
    }
    else
    {
      std::cerr << "setCartDamping only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void LWR4pRobot::startJointPositionController()
  {
    // wait one tick
    FRI->WaitForKRCTick();
    std::cout << "[JointPosController::startController] Starting joint position control." << std::endl;
    int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
    // if there s a problem
    if ((ResultValue != 0) && (ResultValue != EALREADY)) {
      std::cout << "[JointPosController::startController] "
                << "An error occurred during starting up the robot. I will stop controller." << std::endl;
      stopController();
      return;
    }
    this->mode = lwr4p::Mode::JOINT_POS_CTRL;
    std::cout << "[JointPosController::startController] " << "Finished" << std::endl;
  }

  void LWR4pRobot::startJointTorqueController()
  {
    // wait one tick
    FRI->WaitForKRCTick();
    // temp variables
    static float stiffness[7];
    static float stiffnessCart[6];
    static float damping[7];
    static float dampingCart[6];
    static float torques[7];
    static float q[7];
    // put zeros everywhere
    for (int i = 0; i < 7; i++) {
      stiffness[i] = 0;
      damping[i] = 0;
      torques[i] = 0;
    }
    for (int i = 0; i < 6; i++) {
      stiffnessCart[i] = 0;
      dampingCart[i] = 0;
    }

    // set stiffness to zero
    FRI->SetCommandedJointStiffness(stiffness);
    // set stiffness to zero
    FRI->SetCommandedCartStiffness(stiffnessCart);
    // set damping to zero
    FRI->SetCommandedJointDamping(damping);
    // set damping to zero
    FRI->SetCommandedCartDamping(dampingCart);
    // set additional torques to zero
    FRI->SetCommandedJointTorques(torques);
    // set commanded joint positions to current
    FRI->GetCommandedJointPositions(q);
    FRI->SetCommandedJointPositions(q);
    std::cout << "[KukaTorqueController::startController] Starting torque control." << std::endl;
    int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
    // if there s a problem

    if ((ResultValue != 0) && (ResultValue != EALREADY))
    {
      std::cout << "[KukaTorqueController::startController] "
                << "An error occurred during starting up the robot. I will stop controller." << std::endl;
      stopController();
      return;
    }

    this->mode = lwr4p::Mode::JOINT_TORQUE_CTRL;

    std::cout << "[KukaTorqueController::startController] " << "Finished" << std::endl;
  }

  void LWR4pRobot::startCartImpController()
  {
    // wait one tick
    FRI->WaitForKRCTick();
    // temp variables
    static float stiffness[7];
    static float stiffnessCart[6];
    static float damping[7];
    static float dampingCart[6];
    static float torques[7];
    static float p[12];
    // put zeros everywhere
    for (int i = 0; i < 7; i++) {
      stiffness[i] = 0;
      damping[i] = 0;
      torques[i] = 0;
    }
    for (int i = 0; i < 6; i++) {
      stiffnessCart[i] = 0;
      dampingCart[i] = 0;
    }
    // set stiffness to zero
    FRI->SetCommandedJointStiffness(stiffness);
    // set stiffness to zero
    FRI->SetCommandedCartStiffness(stiffnessCart);
    // set damping to zero
    FRI->SetCommandedJointDamping(damping);
    // set damping to zero
    FRI->SetCommandedCartDamping(dampingCart);
    // set additional torques to zero
    FRI->SetCommandedJointTorques(torques);
    // set commanded pose to current (mirror values)
    FRI->GetMeasuredCartPose(p);
    FRI->SetCommandedCartPose(p);
    std::cout << "[KukaCartImpedanceController::startController] Starting Cartesian Impedance control." << std::endl;
    int ResultValue = FRI->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL);
    // if there is a problem

    if ((ResultValue != 0) && (ResultValue != EALREADY))
    {
      std::cout << "[KukaCartImpedanceController::startController] "
                << "An error occurred during starting up the robot. I will stop controller." << std::endl;
      stopController();
      return;
    }

    this->mode = lwr4p::Mode::IMPEDANCE_CONTROL;

    std::cout << "[KukaCartImpedanceController::startController] " << "Finished" << std::endl;
  }

  void LWR4pRobot::stopController()
  {
    if (this->mode != lwr4p::Mode::STOPPED)
    {
      FRI->WaitForKRCTick();
      // printouts
      std::cout << "[KukaController::stopController] Stopping  control." << std::endl;

      static float pose[12];
      static float poseoff[12];

      static float q[7];
      static float qoff[7];
      static float torques[7];

      // set commanded joint positions to current commanded
      FRI->GetCommandedJointPositions(q);
      FRI->GetCommandedJointPositionOffsets(qoff);

      for (int i = 0; i < 7; i++)
      {
        q[i] += qoff[i];
        torques[i] = 0.0;
      }

      FRI->SetCommandedJointPositions(q);

      // set commanded pose  to current commanded
      FRI->GetCommandedCartPose(pose);
      FRI->GetCommandedCartPoseOffsets(poseoff);
      for (int i = 0; i < 12; i++)
      {
        pose[i] += poseoff[i];
      }
      FRI->SetCommandedCartPose(pose);

      // set joint torques to zero
      FRI->SetCommandedJointTorques(torques);

      std::cout << "StopRobot...\n";
      // call stanford command
      FRI->StopRobot();
      this->mode = lwr4p::Mode::STOPPED;
      std::cout << "Done!\n";
    }
    // lower the flag
    std::cout << "[KukaController::stopController] " << "Finished" << std::endl;
  }

  void LWR4pRobot::saveLastJointPosition(float input[7])
  {
    for (size_t i = 0; i < 7; i++)
    {
      last_jnt_pos[i] = input[i];
    }
  }

  void LWR4pRobot::saveLastJointPosition()
  {
    static float temp[7];
    FRI->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < 7; i++)
    {
      last_jnt_pos[i] = temp[i];
    }
  }



}  // namespace lwr
