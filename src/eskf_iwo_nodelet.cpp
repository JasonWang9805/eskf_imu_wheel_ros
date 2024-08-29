//
// Created by jinxu on 2024/08/06.
//
// #include "../include/eskf_iwo_nodelet.h"
#include <eskf_iwo/eskf_iwo_nodelet.h>

namespace eskf_iwo 
{
  void EskfIwoNodelet::onInit() 
  {
    eskf_iwo_ptr.reset(new EskfIwo(getPrivateNodeHandle()));
    if (!eskf_iwo_ptr->initialize()) 
    {
      ROS_ERROR("cannot initialize ESKF IWO...");
      return;
    }
    return;
  }

  PLUGINLIB_EXPORT_CLASS(eskf_iwo::EskfIwoNodelet, nodelet::Nodelet);

} // end namespace 