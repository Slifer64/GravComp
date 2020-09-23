#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>
#include <Eigen/Dense>

#include <robot_wrapper/robot.h>
#include <grav_comp/gui/mainwindow.h>
#include <grav_comp/utils/tf_pose_publisher.h>

class GravComp
{

public:
  GravComp();
  ~GravComp();

  void launch();
protected:

  friend MainWindow;

  bool rec_predef_poses_run;
  ExecResultMsg recPredefPoses();
  ExecResultMsg recordCurrentWrenchQuat();
  ExecResultMsg clearWrenchQuatData();

  ExecResultMsg calcCoM();
  ExecResultMsg loadCoMData(const std::string &path);
  ExecResultMsg saveCoMData(const std::string &save_path);
  ExecResultMsg loadWrenchOrientData(const std::string &path);
  ExecResultMsg saveWrenchOrientData(const std::string &path);

  void launchPublishEEPose(unsigned pub_rate_ms = 200);

  void checkRobot();
  void setMode(rw_::Mode mode);

  std::string err_msg;
  void setErrMsg(const std::string &msg) { err_msg = msg; }
  std::string getErrMsg() const { return err_msg; }

  std::string info_msg;
  void setInfoMsg(const std::string &msg) { info_msg = msg; }
  std::string getInfoMsg() const { return info_msg; }

  std::vector<Eigen::Vector6d> Wrench_data;
  std::vector<Eigen::Quaterniond> Quat_data;
  arma::vec CoM;
  double mass;
  robo_::ToolEstimator tool_estimator;
  bool is_CoM_calculated;

  // robot
  std::string robot_type;
  std::shared_ptr<rw_::Robot> robot;

  // GUI
  MainWindow *gui;

  thr_::Semaphore start_sem;
  thr_::Semaphore finish_sem;

  std::shared_ptr<TfPosePublisher> ee_tf_pub;
  std::shared_ptr<TfPosePublisher> com_tf_pub;

  static void closeGUI(int);
  static MainWindow *gui_; // used to emit closeGUI signal

};

#endif // GRAVITY_COMPENSATION_H
