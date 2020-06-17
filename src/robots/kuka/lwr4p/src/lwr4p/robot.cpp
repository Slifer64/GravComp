/**
 * Copyright (C) 2016 AUTH-lwr
 */

#include <lwr4p/robot.h>
#include <iostream>
#include <string>

#include <ros/package.h>

namespace lwr4p
{

  Robot::Robot(): N_JOINTS(7)
  {
    mode_name[Mode::UNDEFINED] = "UNDEFINED";
    mode_name[Mode::STOPPED] = "STOPPED";
    mode_name[Mode::JOINT_POS_CTRL] = "JOINT_POS_CTRL";
    mode_name[Mode::JOINT_VEL_CTRL] = "JOINT_VEL_CTRL";
    mode_name[Mode::JOINT_TORQUE_CTRL] = "JOINT_TORQUE_CTRL";
    mode_name[Mode::IMPEDANCE_CONTROL] = "IMPEDANCE_CONTROL";
  }

  Robot::~Robot()
  {}

  arma::mat Robot::get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
  {
    arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

    if (t < 0)
    {
      // before start
      retTemp.col(0) = p0;
    }
    else if (t > totalTime)
    {
      // after the end
      retTemp.col(0) = pT;
    }
    else
    {
      // somewhere betweeen ...
      // position
      retTemp.col(0) = p0 +
                       (pT - p0) * (10 * pow(t / totalTime, 3) -
                       15 * pow(t / totalTime, 4) +
                       6 * pow(t / totalTime, 5));
      // vecolity
      retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                       60 * pow(t, 3) / pow(totalTime, 4) +
                       30 * pow(t, 4) / pow(totalTime, 5));
      // acceleration
      retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                       180 * pow(t, 2) / pow(totalTime, 4) +
                       120 * pow(t, 3) / pow(totalTime, 5));
    }

    // return vector
    return retTemp;
  }


}  // namespace lwr
