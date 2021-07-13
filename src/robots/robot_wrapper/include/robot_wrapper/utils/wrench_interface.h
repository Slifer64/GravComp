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

  void initWrenchInterface(std::function<arma::mat()> getTaskRotm);

  void setToolEstimator(const std::string &tool_massCoM_file);
  void setToolEstimator(const robo_::ToolEstimator &tool_est_);

  void useAtiSensor(const std::string &ati_ip);
  // virtual void setWrenchBias() { if (ftsensor) ftsensor->setBias(); }

  void biasAtiSensor() { if (ftsensor) ftsensor->setBias(); }
  virtual void biasRobotSensor() { throw std::runtime_error("[WrenchInterface::biasRobotSensor]: Must be implemented from the corresponding robot...\n"); }

  arma::vec getTaskWrench() const
  { return applyFextDeadZone(get_wrench_fun()); }

  arma::vec getCompTaskWrench() const;

  arma::vec getTaskForce() const
  { return getTaskWrench().subvec(0,2); }

  arma::vec getTaskTorque() const
  { return getTaskWrench().subvec(3,5); }

  void setWrenchDeadZone(const arma::vec &Fext_dead_zone) { this->Fext_dead_zone = Fext_dead_zone; }

protected:

  virtual arma::vec getTaskWrenchFromRobot() const = 0;

  arma::vec getTaskWrenchFromAti() const;
  std::function<arma::vec()> get_wrench_fun;

  std::function<arma::mat()> get_task_rotmat;

  std::shared_ptr<robo_::ToolEstimator> tool_estimator;

  arma::vec applyFextDeadZone(const arma::vec &F_ext) const;

  arma::vec Fext_dead_zone;

  std::shared_ptr<ati::FTSensor> ftsensor;
};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_WRENCH_INTERFACE_H
