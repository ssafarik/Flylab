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

#include "flysim/fly.h"

#include <wx/wx.h>

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace flysim
{

  Fly::Fly(const ros::NodeHandle& nh, const wxImage& fly_image, const Vector2& pos, float orient, wxColour pen_color)
    : nh_(nh)
    , fly_image_(fly_image)
    , pos_(pos)
    , orient_(orient)
    , lin_vel_(0.0)
    , ang_vel_(0.0)
    , pen_on_(true)
    , pen_(pen_color)
    // , pen_(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
  {
    pen_.SetWidth(3);
    fly_ = wxBitmap(fly_image_);

    velocity_sub_ = nh_.subscribe("command_velocity", 1, &Fly::velocityCallback, this);
    pose_pub_ = nh_.advertise<Pose>("pose", 1);
    color_pub_ = nh_.advertise<Color>("color_sensor", 1);
    set_pen_srv_ = nh_.advertiseService("set_pen", &Fly::setPenCallback, this);
    teleport_relative_srv_ = nh_.advertiseService("teleport_relative", &Fly::teleportRelativeCallback, this);
    teleport_absolute_srv_ = nh_.advertiseService("teleport_absolute", &Fly::teleportAbsoluteCallback, this);

    // pixels_per_mm_ = fly_.GetHeight();
    // pixels_per_mm_ = fly_.GetHeight()/4;
    pixels_per_mm_ = 10;
  }


  void Fly::velocityCallback(const VelocityConstPtr& vel)
  {
    last_command_time_ = ros::WallTime::now();
    lin_vel_ = vel->linear;
    ang_vel_ = vel->angular;
  }

  bool Fly::setPenCallback(flysim::SetPen::Request& req, flysim::SetPen::Response&)
  {
    pen_on_ = !req.off;
    if (req.off)
      {
        return true;
      }

    wxPen pen(wxColour(req.r, req.g, req.b));
    if (req.width != 0)
      {
        pen.SetWidth(req.width);
      }

    pen_ = pen;
    return true;
  }

  bool Fly::teleportRelativeCallback(flysim::TeleportRelative::Request& req, flysim::TeleportRelative::Response&)
  {
    teleport_requests_.push_back(TeleportRequest(0, 0, req.angular, req.linear, true));
    return true;
  }

  bool Fly::teleportAbsoluteCallback(flysim::TeleportAbsolute::Request& req, flysim::TeleportAbsolute::Response&)
  {
    teleport_requests_.push_back(TeleportRequest(req.x, req.y, req.theta, 0, false));
    return true;
  }

  void Fly::update(double dt, wxMemoryDC& path_dc, const wxImage& path_image, wxColour background_color, float canvas_width, float canvas_height)
  {
    // first process any teleportation requests, in order
    V_TeleportRequest::iterator it = teleport_requests_.begin();
    V_TeleportRequest::iterator end = teleport_requests_.end();
    for (; it != end; ++it)
      {
        const TeleportRequest& req = *it;

        Vector2 old_pos = pos_;
        if (req.relative)
          {
            orient_ += req.theta;
            pos_.x += sin(orient_ + PI/2.0) * req.linear;
            pos_.y += cos(orient_ + PI/2.0) * req.linear;
          }
        else
          {
            pos_.x = req.pos.x;
            pos_.y = std::max(0.0f, canvas_height - req.pos.y);
            orient_ = req.theta;
          }

        path_dc.SetPen(pen_);
        path_dc.DrawLine(pos_.x * pixels_per_mm_, pos_.y * pixels_per_mm_, old_pos.x * pixels_per_mm_, old_pos.y * pixels_per_mm_);
      }

    teleport_requests_.clear();

    if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(1.0))
      {
        lin_vel_ = 0.0f;
        ang_vel_ = 0.0f;
      }

    Vector2 old_pos = pos_;

    orient_ = fmod(orient_ + ang_vel_ * dt, 2*PI);
    pos_.x += sin(orient_ + PI/2.0) * lin_vel_ * dt;
    pos_.y += cos(orient_ + PI/2.0) * lin_vel_ * dt;

    // Clamp to screen size
    if (pos_.x < 0 || pos_.x >= canvas_width
        || pos_.y < 0 || pos_.y >= canvas_height)
      {
        ROS_WARN("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x, pos_.y);
      }

    pos_.x = std::min(std::max(pos_.x, 0.0f), canvas_width);
    pos_.y = std::min(std::max(pos_.y, 0.0f), canvas_height);

    int canvas_x = pos_.x * pixels_per_mm_;
    int canvas_y = pos_.y * pixels_per_mm_;

    {
      wxImage rotated_image = fly_image_.Rotate(orient_ - PI/2.0, wxPoint(fly_image_.GetWidth() / 2, fly_image_.GetHeight() / 2));

      for (int y = 0; y < rotated_image.GetHeight(); ++y)
        {
          for (int x = 0; x < rotated_image.GetWidth(); ++x)
            {
              if (rotated_image.GetRed(x, y) == 255 && rotated_image.GetBlue(x, y) == 255 && rotated_image.GetGreen(x, y) == 255)
                {
                  rotated_image.SetAlpha(x, y, 0);
                }
            }
        }

      fly_ = wxBitmap(rotated_image);
    }

    Pose p;
    p.x = pos_.x;
    p.y = canvas_height - pos_.y;
    p.theta = orient_;
    p.linear_velocity = lin_vel_;
    p.angular_velocity = ang_vel_;
    pose_pub_.publish(p);

    // Figure out (and publish) the color underneath the fly
    {
      wxSize fly_size = wxSize(fly_.GetWidth(), fly_.GetHeight());
      Color color;
      color.r = path_image.GetRed(canvas_x, canvas_y);
      color.g = path_image.GetGreen(canvas_x, canvas_y);
      color.b = path_image.GetBlue(canvas_x, canvas_y);
      color_pub_.publish(color);
    }

    ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f", nh_.getNamespace().c_str(), pos_.x, pos_.y, orient_);

    if (pen_on_)
      {
        if (pos_ != old_pos)
          {
            path_dc.SetPen(pen_);
            path_dc.DrawLine(pos_.x * pixels_per_mm_, pos_.y * pixels_per_mm_, old_pos.x * pixels_per_mm_, old_pos.y * pixels_per_mm_);
          }
      }
  }

  void Fly::paint(wxDC& dc)
  {
    wxSize fly_size = wxSize(fly_.GetWidth(), fly_.GetHeight());
    dc.DrawBitmap(fly_, pos_.x * pixels_per_mm_ - (fly_size.GetWidth() / 2), pos_.y * pixels_per_mm_ - (fly_size.GetHeight() / 2), true);
  }

}
