/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief ADSB Plugin
 *
 * This plugin publishes ADSB data to simulate manned aircraft and UTM
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#ifndef _GAZEBO_ADSB_PLUGIN_HH_
#define _GAZEBO_ADSB_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>

#include <SITLAdsb.pb.h>

namespace gazebo
{
class AdsbPlugin : public WorldPlugin
{
public:
  AdsbPlugin();
  virtual ~AdsbPlugin();

protected:
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  std::string namespace_;
  event::ConnectionPtr updateConnection_;

  physics::WorldPtr world_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr adsb_pub_;

  sensor_msgs::msgs::SITLAdsb adsb_msg;

  common::Time last_time_;

  double vehicle_lat = 47.397742 * M_PI / 180.0;  // rad
  double vehicle_lon = 8.545594 * M_PI / 180.0;   // rad
  double vehicle_alt = 488.0;                     // meters

  static constexpr double interval_ = 1.0; // 1hz

};     // class GAZEBO_WORLD AdsbPlugin
}      // namespace gazebo
#endif // _GAZEBO_ADSB_PLUGIN_HH_
