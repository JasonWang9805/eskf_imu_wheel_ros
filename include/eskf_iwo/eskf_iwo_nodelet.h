//
// Created by jinxu on 2024/08/06.
//

#ifndef ESKF_IWO_NODELET_H
#define ESKF_IWO_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "run_eskf_gins_wjx.h"

namespace eskf_iwo
{
  class EskfIwoNodelet : public nodelet::Nodelet
  {
    public:
      EskfIwoNodelet() { return; }
      ~EskfIwoNodelet() { return; }

    private:
      virtual void onInit();
      EskfIwoPtr eskf_iwo_ptr;
  };
} // end namespace eskf_iwo

#endif