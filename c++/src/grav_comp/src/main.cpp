#include <grav_comp/grav_comp.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grav_comp");

  std::shared_ptr<GravComp> grav_comp;
  grav_comp.reset(new GravComp());

  grav_comp->run();

  return 0;
}
