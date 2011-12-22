///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////

// $Id: camera1394v2.cpp 30349 2010-06-19 16:12:08Z joq $

#include <signal.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include "dev_camera1394v2.h"
#include "camera1394v2/Camera1394v2Config.h"

/** @file

    @brief ROS driver node for IEEE 1394 digital cameras.

    This is a ROS port of the Player driver for 1394 cameras, using
    libdc1394.  It provides a reliable driver with minimal dependencies,
    intended to fill a role in the ROS image pipeline similar to the other
    ROS camera drivers.

    The ROS image pipeline provides Bayer filtering at a higher level (in
    image_proc).  In some cases it is useful to run the driver without the
    entire image pipeline, so libdc1394 Bayer decoding is also provided.

    @par Advertises

    - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

    - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
    information for each image.

*/

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  ROS_ERROR("Segmentation fault, stopping camera.");
  ros::shutdown();                      // stop the main loop
}

class Camera1394v2Node
{
private:

  Driver::state_t state_;               // current driver state

  ros::NodeHandle privNH_;              // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  sensor_msgs::Image image_;
  sensor_msgs::CameraInfo cam_info_;
  std::string camera_name_;             // camera name

  /** 1394 camera device interface */
  camera1394v2::Camera1394v2 *dev_;

  /** dynamic parameter configuration */
  typedef camera1394v2::Camera1394v2Config Config;
  Config config_;

  /** camera calibration information */
  CameraInfoManager *cinfo_;
  bool calibration_matches_;            // cam_info_ matches video mode

  /** image transport interfaces */
  image_transport::ImageTransport *it_;
  image_transport::CameraPublisher image_pub_;

public:

  Camera1394v2Node(): it_(0)
  {
    state_ = Driver::CLOSED;
    privNH_ = ros::NodeHandle("~");
    camera_nh_ = ros::NodeHandle("camera");
    camera_name_ = "camera";
    cinfo_ = new CameraInfoManager(camera_nh_);
    dev_ = new camera1394v2::Camera1394v2();
    calibration_matches_ = true;
  }

  ~Camera1394v2Node()
  {
    if (it_)
      delete it_;
    delete dev_;
    delete cinfo_;
  }

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   */
  bool openCamera(Config &newconfig)
  {
    bool success = true;
    try
      {
        if (dev_->open(newconfig.guid.c_str(), newconfig.video_mode.c_str(),
                       newconfig.frame_rate, newconfig.iso_speed,
                       newconfig.bayer_pattern.c_str(),
                       newconfig.bayer_method.c_str())
            == 0)
          {
            if (camera_name_ != dev_->device_id_)
              {
                camera_name_ = dev_->device_id_;
                if (!cinfo_->setCameraName(camera_name_))
                  {
                    // GUID is 16 hex digits, which should be valid.
                    // If not, use it for log messages anyway.
                    ROS_WARN_STREAM("[" << camera_name_ << "] name not valid"
                                    << " for camera_info_manger");
                  }
              }
            ROS_INFO_STREAM("[" << camera_name_
                            << "] opened: " << newconfig.video_mode << ", "
                            << newconfig.frame_rate << " fps, "
                            << newconfig.iso_speed << " Mb/s");
            state_ = Driver::OPENED;
            calibration_matches_ = true;
          }

      }
    catch (camera1394v2::Exception& e)
      {
        ROS_FATAL_STREAM("[" << camera_name_
                         << "] exception opening device: " << e.what());
        success = false;
      }

    return success;
  }

  /** Publish camera stream topics
   *
   *  @pre image_ contains latest camera frame
   */
  void publish()
  {
    image_.header.frame_id = config_.frame_id;

    // get current CameraInfo data
    cam_info_ = cinfo_->getCameraInfo();

    if (cam_info_.height != image_.height || cam_info_.width != image_.width)
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches_)
          {
            // warn user once
            calibration_matches_ = false;
            ROS_WARN_STREAM("[" << camera_name_
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
        cam_info_ = sensor_msgs::CameraInfo();
        cam_info_.height = image_.height;
        cam_info_.width = image_.width;
      }
    else if (!calibration_matches_)
      {
        // calibration OK now
        calibration_matches_ = true;
        ROS_INFO_STREAM("[" << camera_name_
                        << "] calibration matches video mode now");
      }

    cam_info_.header.frame_id = config_.frame_id;
    cam_info_.header.stamp = image_.header.stamp;

    // @todo log a warning if (filtered) time since last published
    // image is not reasonably close to configured frame_rate

    // Publish via image_transport
    image_pub_.publish(image_, cam_info_);
  }

  /** Read camera data.
   *
   * @return true if successful
   */
  bool read()
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        dev_->readData(image_);
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (camera1394v2::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void reconfig(Config &newconfig, uint32_t level)
  {
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "camera";
    std::string tf_prefix = tf::getPrefixParam(privNH_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    if (state_ == Driver::CLOSED)
      {
        // open with new values
        if (openCamera(newconfig))
          {
            // update GUID string parameter
            newconfig.guid = camera_name_;
          }
      }

    if (config_.camera_info_url != newconfig.camera_info_url)
      {
        // set the new URL and load CameraInfo (if any) from it
        if (cinfo_->validateURL(newconfig.camera_info_url))
          {
            cinfo_->loadCameraInfo(newconfig.camera_info_url);
          }
        else
          {
            // new URL not valid, use the old one
            newconfig.camera_info_url = config_.camera_info_url;
          }
      }

    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
      {
        // configure IIDC features
        if (level & Levels::RECONFIGURE_CLOSE)
          {
            // initialize all features for newly opened device
            if (false == dev_->features_->initialize(&newconfig))
              {
                ROS_ERROR_STREAM("[" << camera_name_
                                 << "] feature initialization failure");
                closeCamera();          // can't continue
              }
          }
        else
          {
            // update any features that changed
            dev_->features_->reconfigure(&newconfig);
          }
      }

    config_ = newconfig;                // save new parameters

    ROS_DEBUG_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url " << newconfig.camera_info_url);
  }


  /** driver main spin loop */
  void spin(void)
  {
    // the bring up order is tricky
    ros::NodeHandle node;

    // define segmentation fault handler, sometimes libdc1394 craps out
    signal(SIGSEGV, &sigsegv_handler);

    // Define dynamic reconfigure callback, which gets called
    // immediately with level 0xffffffff.  The reconfig() method will
    // set initial parameter values, then open the device if it can.
    dynamic_reconfigure::Server<Config> srv;
    dynamic_reconfigure::Server<Config>::CallbackType f
      = boost::bind(&Camera1394v2Node::reconfig, this, _1, _2);
    srv.setCallback(f);

    // set up ROS interfaces in camera namespace
    it_ = new image_transport::ImageTransport(camera_nh_);
    image_pub_ = it_->advertiseCamera("image_raw", 1);

    while (node.ok())
      {
        if (state_ != Driver::CLOSED)
          {
            if (read())
              {
                publish();
              }
          }

        ros::spinOnce();
      }

    closeCamera();
  }

}; // end Camera1394v2Node class definition


/** Main entry point */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera1394v2_node");
  ros::NodeHandle node;
  Camera1394v2Node cm;

  cm.spin();

  return 0;
}
