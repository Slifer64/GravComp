#include <robo_lib/joint_state_publisher.h>

namespace as64_
{

namespace robo_
{

JointStatePublisher::JointStatePublisher()
{
  run = false;
}

JointStatePublisher::~JointStatePublisher()
{
  stop();
}

void JointStatePublisher::start()
{
  if (run == true) return;

  joint_state_pub = nh.advertise<sensor_msgs::JointState>(pub_topic, 1);
  run = true;
  run_thread = std::thread(&JointStatePublisher::publishLoop, this);
}

void JointStatePublisher::stop()
{
  run = false;
  if (run_thread.joinable()) run_thread.join();
  joint_state_pub.shutdown();
}

void JointStatePublisher::addFun(void (*fun)(sensor_msgs::JointState &))
{
  std::unique_lock<std::mutex> lck(mtx);
  add_joint_state_funs.push_back( std::bind(fun, std::placeholders::_1) );
}


void JointStatePublisher::setPublishCycle(double Ts)
{
  this->Ts = Ts*1e9;
}

void JointStatePublisher::setPublishTopic(const std::string &pub_topic)
{
  this->pub_topic = pub_topic;
}


void JointStatePublisher::publishLoop()
{
  while (run)
  {
    std::unique_lock<std::mutex> lck(mtx);

    sensor_msgs::JointState joint_state_msg;
    for (int i=0; i<add_joint_state_funs.size(); i++) (add_joint_state_funs[i])(joint_state_msg);

//     std::cout << "==========================================\n";
//     std::cout << "==========  Joint State msg ==============\n";
//     int N = joint_state_msg.name.size();
//     std::cout << "** names **\n";
//     for (int i=0;i<N; i++)
//     {
//       std::cout << joint_state_msg.name[i] << " ";
//     }
//     std::cout << "\n** position **\n";
//     for (int i=0;i<N; i++)
//     {
//       std::cout << joint_state_msg.position[i] << " ";
//     }
//     std::cout << "\n";

    joint_state_pub.publish(joint_state_msg);

    lck.unlock();

    std::this_thread::sleep_for(std::chrono::nanoseconds(Ts));
  }
}

} // namespace robo_

} // namespace as64_
