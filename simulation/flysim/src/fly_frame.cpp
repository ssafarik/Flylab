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

#include "flysim/fly_frame.h"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0xff
#define DEFAULT_BG_G 0xff
#define DEFAULT_BG_B 0xff
// #define DEFAULT_BG_R 0x45
// #define DEFAULT_BG_G 0x56
// #define DEFAULT_BG_B 0xff

namespace flysim
{

  FlyFrame::FlyFrame(wxWindow* parent)
    : wxFrame(parent, wxID_ANY, wxT("FlySim"), wxDefaultPosition, wxSize(600, 600), wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER)
    , frame_count_(0)
    , id_counter_(0)
  {
    srand(time(NULL));

    update_timer_ = new wxTimer(this);
    update_timer_->Start(16);

    Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(FlyFrame::onUpdate), NULL, this);
    Connect(wxEVT_PAINT, wxPaintEventHandler(FlyFrame::onPaint), NULL, this);

    nh_.setParam("background_r", DEFAULT_BG_R);
    nh_.setParam("background_g", DEFAULT_BG_G);
    nh_.setParam("background_b", DEFAULT_BG_B);

    std::string flies[2] =
      {
        "robot.png",
        "fly.png"
      };

    std::string images_path = ros::package::getPath("flysim") + "/images/";
    for (size_t i = 0; i < 2; ++i)
      {
        fly_images_[i].LoadFile(wxString::FromAscii((images_path + flies[i]).c_str()));
        fly_images_[i].SetMask(true);
        fly_images_[i].SetMaskColour(255, 255, 255);
      }

    // pixels_per_mm_ = fly_images_[0].GetHeight();
    // pixels_per_mm_ = fly_images_[0].GetHeight()/4;
    pixels_per_mm_ = 10;

    path_bitmap_ = wxBitmap(GetSize().GetWidth(), GetSize().GetHeight());
    path_dc_.SelectObject(path_bitmap_);
    clear();

    clear_srv_ = nh_.advertiseService("clear", &FlyFrame::clearCallback, this);
    reset_srv_ = nh_.advertiseService("reset", &FlyFrame::resetCallback, this);
    spawn_srv_ = nh_.advertiseService("spawn", &FlyFrame::spawnCallback, this);
    kill_srv_ = nh_.advertiseService("kill", &FlyFrame::killCallback, this);

    ROS_INFO("Starting flysim with node name %s", ros::this_node::getName().c_str()) ;

    width_in_mm_ = GetSize().GetWidth() / pixels_per_mm_;
    height_in_mm_ = GetSize().GetHeight() / pixels_per_mm_;
    spawnFly("robot", width_in_mm_ / 2.0, height_in_mm_ / 2.0, 0);
  }

  FlyFrame::~FlyFrame()
  {
    delete update_timer_;
  }

  bool FlyFrame::spawnCallback(flysim::Spawn::Request& req, flysim::Spawn::Response& res)
  {
    std::string name = spawnFly(req.name, req.x, req.y, req.theta);
    if (name.empty())
      {
        ROS_ERROR("A fly named [%s] already exists", req.name.c_str());
        return false;
      }

    res.name = name;

    return true;
  }

  bool FlyFrame::killCallback(flysim::Kill::Request& req, flysim::Kill::Response&)
  {
    M_Fly::iterator it = flies_.find(req.name);
    if (it == flies_.end())
      {
        ROS_ERROR("Tried to kill fly [%s], which does not exist", req.name.c_str());
        return false;
      }

    flies_.erase(it);

    return true;
  }

  bool FlyFrame::hasFly(const std::string& name)
  {
    return flies_.find(name) != flies_.end();
  }

  std::string FlyFrame::spawnFly(const std::string& name, float x, float y, float angle)
  {
    std::string real_name = name;
    if (real_name.empty())
      {
        do
          {
            std::stringstream ss;
            if (id_counter_ == 0)
              {
                ss << "fly";
                id_counter_++;
              }
            else
              {
                ss << "fly" << ++id_counter_;
              }
            real_name = ss.str();
          } while (hasFly(real_name));
      }
    else
      {
        if (hasFly(real_name))
          {
            return "";
          }
      }

    int image_n = 1;
    wxColour pen_color = wxColour(0x00,0xff,0x00);
    if (flies_.empty())
      {
        image_n = 0;
        pen_color = wxColour(0x00,0x00,0xff);
      }

    // FlyPtr t(new Fly(ros::NodeHandle(real_name), fly_images_[rand() % 2], Vector2(x, y), angle));
    FlyPtr t(new Fly(ros::NodeHandle(real_name), fly_images_[image_n], Vector2(x, y), angle, pen_color));
    flies_[real_name] = t;

    ROS_INFO("Spawning fly [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

    return real_name;
  }

  void FlyFrame::clear()
  {
    int r = DEFAULT_BG_R;
    int g = DEFAULT_BG_G;
    int b = DEFAULT_BG_B;

    nh_.param("background_r", r, r);
    nh_.param("background_g", g, g);
    nh_.param("background_b", b, b);

    path_dc_.SetBackground(wxBrush(wxColour(r, g, b)));
    path_dc_.Clear();
  }

  void FlyFrame::onUpdate(wxTimerEvent& evt)
  {
    ros::spinOnce();

    updateFlies();

    if (!ros::ok())
      {
        Close();
      }
  }

  void FlyFrame::onPaint(wxPaintEvent& evt)
  {
    wxPaintDC dc(this);

    dc.DrawBitmap(path_bitmap_, 0, 0, true);

    M_Fly::iterator it = flies_.begin();
    M_Fly::iterator end = flies_.end();
    for (; it != end; ++it)
      {
        it->second->paint(dc);
      }
  }

  void FlyFrame::updateFlies()
  {
    if (last_fly_update_.isZero())
      {
        last_fly_update_ = ros::WallTime::now();
        return;
      }

    if (frame_count_ % 3 == 0)
      {
        path_image_ = path_bitmap_.ConvertToImage();
        Refresh();
      }

    M_Fly::iterator it = flies_.begin();
    M_Fly::iterator end = flies_.end();
    for (; it != end; ++it)
      {
        it->second->update(0.016, path_dc_, path_image_, path_dc_.GetBackground().GetColour(), width_in_mm_, height_in_mm_);
      }

    ++frame_count_;
  }


  bool FlyFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    ROS_INFO("Clearing flysim.");
    clear();
    return true;
  }

  bool FlyFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    ROS_INFO("Resetting flysim.");
    flies_.clear();
    id_counter_ = 0;
    spawnFly("robot", width_in_mm_ / 2.0, height_in_mm_ / 2.0, 0);
    clear();
    return true;
  }

}
