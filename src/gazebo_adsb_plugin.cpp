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

#include <gazebo_adsb_plugin.h>

namespace gazebo {
GZ_REGISTER_WORLD_PLUGIN(AdsbPlugin)

AdsbPlugin::AdsbPlugin() : WorldPlugin()
{ }

AdsbPlugin::~AdsbPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
}

void AdsbPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;

  if(sdf->HasElement("Latitude")) {
    double latitude;
    getSdfParam<double>(sdf, "Latitude", latitude, 47.397742);
    vehicle_lat = latitude * M_PI / 180.0;
  }
  if(sdf->HasElement("Longitude")) {
    double longitude;
    getSdfParam<double>(sdf, "Longitude", longitude, 8.545594);
    vehicle_lon = longitude * M_PI / 180.0;
  }
  if(sdf->HasElement("Altitude")) {
    getSdfParam<double>(sdf, "homeAltitude", vehicle_alt, vehicle_alt);
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AdsbPlugin::OnUpdate, this, _1));

  adsb_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLAdsb>("~/transponder", 10);
}

void AdsbPlugin::OnUpdate(const common::UpdateInfo&){
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

  double dt = (current_time - last_time_).Double();

  if (dt > interval_) {
    adsb_msg.set_time_usec(current_time.Double() * 1e6);
    adsb_msg.set_latitude_deg(vehicle_lat * 180.0/ M_PI);
    adsb_msg.set_longitude_deg(vehicle_lon * 180.0/ M_PI);
    adsb_msg.set_altitude(vehicle_alt);
    adsb_msg.set_altitude_type(1); //ADSB_ALTITUDE_TYPE_GEOMETRIC

    last_time_ = current_time;
    adsb_pub_->Publish(adsb_msg);
  }
}
} // namespace gazebo
