#ifndef AS64_ROBOT_WRAPPER_WRENCH_INTERFACE_H
#define AS64_ROBOT_WRAPPER_WRENCH_INTERFACE_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <memory>
#include <armadillo>

#include <ati_sensor/ft_sensor.h>
#include <robo_lib/tool_estimator.h>

using namespace as64_;

namespace rw_
{

class WrenchInterface
{
public:
  WrenchInterface();
  ~WrenchInterface();

  void initWrenchInterface(std::function<arma::mat()> getTaskRotm, const arma::mat &T_sensor_tool = arma::mat().eye(4,4));

  void setToolEstimator(const std::string &tool_massCoM_file);
  void setToolEstimator(const robo_::ToolEstimator &tool_est_);

  void useAtiSensor(const std::string &ati_ip);

  void biasFTsensor() { bias_ftsensor_fun(); }

  arma::vec getTaskWrench() const
  { return applyFextDeadZone(get_wrench_fun()); }

  arma::vec getCompTaskWrench() const;

  arma::vec getTaskForce() const
  { return getTaskWrench().subvec(0,2); }

  arma::vec getTaskTorque() const
  { return getTaskWrench().subvec(3,5); }

  void setWrenchDeadZone(const arma::vec &Fext_dead_zone) { this->Fext_dead_zone = Fext_dead_zone; }

protected:

  // get wrench from Robot 
  virtual arma::vec getTaskWrenchFromRobot() const = 0;
  virtual void biasRobotSensor() { throw std::runtime_error("[WrenchInterface::biasRobotSensor]: Must be implemented from the corresponding robot...\n"); }
  
  // get wrench from Ati
  std::shared_ptr<ati::FTSensor> ftsensor;
  arma::vec getTaskWrenchFromAti() const;
  void biasAtiSensor() { if (ftsensor) ftsensor->setBias(); }


  std::function<arma::vec()> get_wrench_fun;
  std::function<void()> bias_ftsensor_fun;

  std::function<arma::mat()> get_task_rotmat;

  std::shared_ptr<robo_::ToolEstimator> tool_estimator;

  arma::vec applyFextDeadZone(const arma::vec &F_ext) const;

  arma::vec Fext_dead_zone;

  arma::mat R_tool_sensor;
  arma::vec p_tool_sensor;
  arma::vec transferWrenchFromSensorToTool(const arma::vec &F_sensor) const;
 
};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_WRENCH_INTERFACE_H
