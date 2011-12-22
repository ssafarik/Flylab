/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FLYSIM_FLY_H
#define FLYSIM_FLY_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <flysim/Pose.h>
#include <flysim/Velocity.h>
#include <flysim/SetPen.h>
#include <flysim/TeleportRelative.h>
#include <flysim/TeleportAbsolute.h>
#include <flysim/Color.h>

#include <wx/wx.h>

#define PI 3.14159265

namespace flysim
{

  struct Vector2
  {
  Vector2()
  : x(0.0)
  , y(0.0)
    {}

  Vector2(float _x, float _y)
  : x(_x)
  , y(_y)
    {}

    bool operator==(const Vector2& rhs)
    {
      return x == rhs.x && y == rhs.y;
    }

    bool operator!=(const Vector2& rhs)
    {
      return x != rhs.x || y != rhs.y;
    }

    float x;
    float y;
  };

  class Fly
  {
  public:
    Fly(const ros::NodeHandle& nh, const wxImage& fly_image, const Vector2& pos, float orient, wxColour pen_color);

    void update(double dt, wxMemoryDC& path_dc, const wxImage& path_image, wxColour background_color, float canvas_width, float canvas_height);
    void paint(wxDC& dc);
  private:
    void velocityCallback(const VelocityConstPtr& vel);
    bool setPenCallback(flysim::SetPen::Request&, flysim::SetPen::Response&);
    bool teleportRelativeCallback(flysim::TeleportRelative::Request&, flysim::TeleportRelative::Response&);
    bool teleportAbsoluteCallback(flysim::TeleportAbsolute::Request&, flysim::TeleportAbsolute::Response&);

    ros::NodeHandle nh_;

    wxImage fly_image_;
    wxBitmap fly_;

    Vector2 pos_;
    float orient_;

    float lin_vel_;
    float ang_vel_;
    bool pen_on_;
    wxPen pen_;

    ros::Subscriber velocity_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher color_pub_;
    ros::ServiceServer set_pen_srv_;
    ros::ServiceServer teleport_relative_srv_;
    ros::ServiceServer teleport_absolute_srv_;

    ros::WallTime last_command_time_;

    float pixels_per_mm_;

    struct TeleportRequest
    {
    TeleportRequest(float x, float y, float _theta, float _linear, bool _relative)
    : pos(x, y)
    , theta(_theta)
        , linear(_linear)
        , relative(_relative)
      {}

      Vector2 pos;
      float theta;
      float linear;
      bool relative;
    };
    typedef std::vector<TeleportRequest> V_TeleportRequest;
    V_TeleportRequest teleport_requests_;
  };
  typedef boost::shared_ptr<Fly> FlyPtr;

}

#endif
