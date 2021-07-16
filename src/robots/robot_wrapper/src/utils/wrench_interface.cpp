#include <robot_wrapper/utils/wrench_interface.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <ros/package.h>

namespace rw_
{

#define WrenchInterface_fun_ std::string("[WrenchInterface::") + __func__ + "]: "

WrenchInterface::WrenchInterface()
{
  R_tool_sensor = arma::mat().eye(3,3);
  p_tool_sensor = {0,0,0};
}

WrenchInterface::~WrenchInterface()
{

}

void WrenchInterface::initWrenchInterface(std::function<arma::mat()> getTaskRotm, const arma::mat &T_sensor_tool)
{
  R_tool_sensor = T_sensor_tool.submat(0,0,2,2).t();
  p_tool_sensor = R_tool_sensor * T_sensor_tool.submat(0,3,2,3);

  get_task_rotmat = getTaskRotm;

  tool_estimator.reset(new robo_::ToolEstimator);

  get_wrench_fun = std::bind(&WrenchInterface::getTaskWrenchFromRobot, this);

  bias_ftsensor_fun = std::bind(&WrenchInterface::biasRobotSensor, this);

  std::string params_filename = ros::package::getPath("robot_wrapper") + "/config/params.yaml";
  rw_::XmlParser parser(params_filename);

  std::string load_fail_msg = WrenchInterface_fun_ + "Failed to load param ";

  // =======  check whether to use ati-sensor  =======
  bool use_ati = false;
  if (parser.getParam("use_ati", use_ati) && use_ati)
  {
    std::string ati_ip;
    if (!parser.getParam("ati_ip",ati_ip)) throw std::runtime_error(load_fail_msg + "\"ati_ip\"...");
    this->useAtiSensor(ati_ip);
  }

  // check whether to bias sensor for wrench measurements
  bool set_wrench_bias = false;
  if (parser.getParam("set_wrench_bias", set_wrench_bias) && set_wrench_bias)
  {
    if (use_ati) this->biasAtiSensor();
    else this->biasRobotSensor();
  }

  if (!parser.getParam("Fext_dead_zone", Fext_dead_zone)) Fext_dead_zone = arma::vec().zeros(6);

  std::string tool_massCoM_file;
  if (parser.getParam("tool_massCoM_file", tool_massCoM_file)) this->setToolEstimator(ros::package::getPath("robot_wrapper") + "/config/" + tool_massCoM_file);
}


arma::vec WrenchInterface::getCompTaskWrench() const
{
  arma::mat R = get_task_rotmat();

  arma::vec tool_wrench = tool_estimator->getToolWrench(R);

  tool_wrench.subvec(0,2) = R*tool_wrench.subvec(0,2);
  tool_wrench.subvec(3,5) = R*tool_wrench.subvec(3,5);

  return applyFextDeadZone( get_wrench_fun() - tool_wrench );
}

arma::vec WrenchInterface::applyFextDeadZone(const arma::vec &F_ext) const
{
  arma::vec sign_Fext = arma::sign(F_ext);
  arma::vec Fext2 = F_ext - sign_Fext%Fext_dead_zone;
  return 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);
}

void WrenchInterface::setToolEstimator(const std::string &tool_massCoM_file)
{
  tool_estimator.reset(new robo_::ToolEstimator);
  tool_estimator->initFromFile(tool_massCoM_file);
}

void WrenchInterface::setToolEstimator(const robo_::ToolEstimator &tool_est_)
{
  this->tool_estimator->setMass(tool_est_.getMass());
  this->tool_estimator->setCoM(tool_est_.getCoM());
}

void WrenchInterface::useAtiSensor(const std::string &ati_ip)
{
  ftsensor.reset(new ati::FTSensor);
  std::cerr << "=======> Initializing F/T sensor at ip: " << ati_ip << "\n";
  ftsensor->init(ati_ip.c_str());
  ftsensor->setTimeout(1.0);
  // ftsensor->setBias();
  std::cerr << "=======> F/T sensor initialized successfully!\n";

  get_wrench_fun = std::bind(&WrenchInterface::getTaskWrenchFromAti, this);
  bias_ftsensor_fun = std::bind(&WrenchInterface::biasAtiSensor, this);
}

arma::vec WrenchInterface::getTaskWrenchFromAti() const
{
  if (!ftsensor) throw std::runtime_error(WrenchInterface_fun_ + "Ati sensor is not initialized...\n");

  static double msr[6];
  uint32_t rdt(0),ft(0);
  (const_cast<ati::FTSensor *>(ftsensor.get()))->getMeasurements(msr,rdt,ft);
  //ftsensor->getMeasurements(measurements,rdt,ft);

  // transfer sensor wrench to tool frame and express it w.r.t. tool frame
  arma::vec Fext( { msr[0], msr[1], msr[2], msr[3], msr[4], msr[5] } );
  Fext = transferWrenchFromSensorToTool(Fext);

  // express tool wrench w.r.t. base frame
  arma::mat R = get_task_rotmat();
  Fext.subvec(0,2) = R*Fext.subvec(0,2);
  Fext.subvec(3,5) = R*Fext.subvec(3,5);
  return Fext;
}

arma::vec WrenchInterface::transferWrenchFromSensorToTool(const arma::vec &F_sensor) const
{
  arma::vec force_tool = R_tool_sensor*F_sensor.subvec(0,2);
  arma::vec torque_tool = R_tool_sensor*F_sensor.subvec(3,5) + arma::cross(p_tool_sensor, force_tool);

  return arma::join_vert(force_tool, torque_tool);
}


} // namespace rw_
