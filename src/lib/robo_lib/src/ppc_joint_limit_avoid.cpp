
#include <robo_lib/ppc_joint_limit_avoid.h>
#include <stdexcept>

namespace as64_
{

namespace robo_
{


PPCJointLimAvoid::PPCJointLimAvoid()
{

}

PPCJointLimAvoid::PPCJointLimAvoid(const arma::vec &qmin, const arma::vec &qmax)
{
  // initialization
  init(qmin,qmax);
}

void PPCJointLimAvoid::init(const arma::vec &qmin, const arma::vec &qmax)
{
  // check if vectors have the same size
  if (qmax.n_rows != qmin.n_rows)
  {
    throw std::runtime_error("[PPCJointLimAvoid]: Joint limit vectors do not have the same size!\n");
    Njoints_ = -1;
    qmean_(0) = -1;
  }
  else
  {
    // set number of joints
    Njoints_ = qmax.n_rows;

    // initialize joint mean;
    qmean_ = (qmin + qmax) / 2;

    // initialize rho
    rho_ = (qmax - qmin) / 2;
  }

  // set the limits
  qmin_ = qmin;
  qmax_ = qmax;

  // resize gain vector
  kq_.resize(Njoints_);

  gain_ = 1e-4;
  for (int i = 0; i < Njoints_; i++) kq_(i) = gain_;

  // initialize variables
  initAllVarsToZero();
}

void PPCJointLimAvoid::initAllVarsToZero()
{
  e_ = arma::zeros<arma::vec>(Njoints_);
  c_ = arma::zeros<arma::vec>(Njoints_);
  epsilon_ = arma::zeros<arma::vec>(Njoints_);
  dT_ = arma::zeros<arma::vec>(Njoints_);
  sig_ = arma::zeros<arma::vec>(Njoints_);
}

void PPCJointLimAvoid::setLowLimit(int joint_index, double low_limit)
{
  // set the value
  qmin_(joint_index) = low_limit;
}

void PPCJointLimAvoid::setLowLimits(const arma::vec &qmins)
{
  // set the vector values
  qmin_ = qmins;
}

void PPCJointLimAvoid::setHighLimit(int joint_index, double high_limit)
{
  // set the value
  qmax_(joint_index) = high_limit;
}

void PPCJointLimAvoid::setHighLimits(const arma::vec &qmaxs)
{
  // set the vector values
  qmax_ = qmaxs;
}

void PPCJointLimAvoid::setGain(int joint_index, double gain)
{
  // set the value
  kq_(joint_index) = gain;
}

void PPCJointLimAvoid::setGains(double gain)
{
  gain_ = gain;

  // set all vector values to the same value
  for (int i = 0; i < Njoints_; i++) kq_(i) = gain;
}

void PPCJointLimAvoid::setGains(const arma::vec &gains)
{
  // set the vector values
  kq_ = gains;
}

arma::vec PPCJointLimAvoid::getControlSignal(const arma::vec &q_meas)
{
  // compute the error from the mean
  e_ = q_meas - qmean_;

  for (int i = 0; i < Njoints_; i++)
  {

    // compute the intermediate parameter c of controller
    c_(i) = e_(i) / rho_(i);

    // compute espilon of PPC methodology
    epsilon_(i) = std::log((1 + c_(i)) / (1 - c_(i)));

    // compute jacobian of PPC method
    dT_(i) 	=  1 / (e_(i) + rho_(i)) - 1 / (e_(i) - rho_(i));

    // signal for each joint
    sig_(i) = - kq_(i) * dT_(i) * epsilon_(i) ;

  }

  return sig_;
}


}; // namespace robo_

}; // namespace as64_
