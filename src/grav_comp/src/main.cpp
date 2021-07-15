#include <grav_comp/grav_comp.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <gui_lib/utils.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grav_comp");

  std::shared_ptr<GravComp> grav_comp(new GravComp());
  gui_::launchGui([grav_comp](){ return grav_comp->createMainWindow(); }, &(grav_comp->gui_finished));

  return 0;
}
