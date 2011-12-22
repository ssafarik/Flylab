// Copyright (C) 2008-2009 Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>

#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <stdlib.h>

#include "dcam1394/dcam1394.h"
#include <cv_bridge/CvBridge.h>

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#include "dcam1394/dcam1394.h"
// #include <cstring>
// #include <cstdio>

using namespace std;
using namespace ros;

class NewDcam : public dcam::Dcam
{
public:
  NewDcam(uint64_t guid, size_t bufferSize = 8) : dcam::Dcam(guid,bufferSize) {}

  dc1394video_frame_t* getFrame() { return camFrame; }

  // void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
  // {
  //   if (!dcam::dcRef)
  //     throw dcam::DcamException("not ready");
  //   dc1394error_t err = dc1394_feature_set_mode(dcCam, feature, mode);
  //   if( err != DC1394_SUCCESS )
  //     throw dcam::DcamException("error");
  // }
};

class CameraOpenCVNode
{
  typedef void (*fnFillImageData)(void* img, IplImage* frame);
  map<string,dc1394video_mode_t> _mapCameraModes;
  map<dc1394color_coding_t,pair<int, int> > mapcv;
  NodeHandle _node;
  image_transport::ImageTransport _it;
  sensor_msgs::Image _imagemsg;
  sensor_msgs::CameraInfo _undistorted_infomsg, _original_infomsg;
  image_transport::Publisher _pubUndistortedImage, _pubOriginalImage;
  Publisher _pubUndistortedInfo, _pubOriginalInfo;

public:


  boost::shared_ptr<NewDcam> cam;
  dc1394video_mode_t mode;
  string windowname, compression;
  int display;
  double framerate;
  double square_roi;
  double exposure, brightness, contrast, gain, shutter; // if positive, set the values
  double wb_blueu, wb_redv;
  int delay_us;
  int uid;
  IplImage* frame, *frame_undist;
  int cambuffersize;
  uint64_t camguid;
  dc1394color_filter_t bayer;
  bool bEnableBayer;

  // calibration data
  float kc_original[5]; // radial distortion
  float kc_undistorted[5]; // radial distortion
  IplImage* _pUndistortionMapX, *_pUndistortionMapY; // undistortion maps

  // ROI data
  // double roi_undistorted_x_offset,roi_undistorted_y_offset,roi_undistorted_height,roi_undistorted_width;

  CameraOpenCVNode() : _it(_node), frame(NULL), frame_undist(NULL),
                       _pUndistortionMapX(NULL), _pUndistortionMapY(NULL),
                       bToggleVideoWriter(false), bSnapImage(false)
  {
    bEnableBayer = false;
    uid = 1;
    cambuffersize = 0;

    dcam::init();
    int numcams = dcam::numCameras();
    //ROS_ASSERT( numcams > 0 );
    if (numcams == 0)
      {
        ROS_ERROR("No cameras found! Perhaps the camera is not connected? Exiting...\n");
        exit(1);
      }

    _pubUndistortedImage = _it.advertise("UndistortedImage",1);
    _pubOriginalImage = _it.advertise("OriginalImage",1);
    _pubUndistortedInfo = _node.advertise<sensor_msgs::CameraInfo>("UndistortedCameraInfo",4);
    _pubOriginalInfo = _node.advertise<sensor_msgs::CameraInfo>("OriginalCameraInfo",4);
    string smode;

    // format_0
    _mapCameraModes["MODE_160x120_YUV444"] = DC1394_VIDEO_MODE_160x120_YUV444;
    _mapCameraModes["MODE_320x240_YUV422"] = DC1394_VIDEO_MODE_320x240_YUV422;
    _mapCameraModes["MODE_640x480_YUV411"] = DC1394_VIDEO_MODE_640x480_YUV411;
    _mapCameraModes["MODE_640x480_YUV422"] = DC1394_VIDEO_MODE_640x480_YUV422;
    _mapCameraModes["MODE_640x480_RGB"] = DC1394_VIDEO_MODE_640x480_RGB8;
    _mapCameraModes["MODE_640x480_MONO"] = DC1394_VIDEO_MODE_640x480_MONO8;
    _mapCameraModes["MODE_640x480_MONO16"] = DC1394_VIDEO_MODE_640x480_MONO16;
    // format_1
    _mapCameraModes["MODE_800x600_YUV422"] = DC1394_VIDEO_MODE_800x600_YUV422;
    _mapCameraModes["MODE_800x600_RGB"] = DC1394_VIDEO_MODE_800x600_RGB8;
    _mapCameraModes["MODE_800x600_MONO"] = DC1394_VIDEO_MODE_800x600_MONO8;
    _mapCameraModes["MODE_1024x768_YUV422"] = DC1394_VIDEO_MODE_1024x768_YUV422;
    _mapCameraModes["MODE_1024x768_RGB"] = DC1394_VIDEO_MODE_1024x768_RGB8;
    _mapCameraModes["MODE_1024x768_MONO"] = DC1394_VIDEO_MODE_1024x768_MONO8;
    _mapCameraModes["MODE_800x600_MONO16"] = DC1394_VIDEO_MODE_800x600_MONO16;
    _mapCameraModes["MODE_1024x768_MONO16"] = DC1394_VIDEO_MODE_1024x768_MONO16;
    // format_2
    _mapCameraModes["MODE_1280x960_YUV422"] = DC1394_VIDEO_MODE_1280x960_YUV422;
    _mapCameraModes["MODE_1280x960_RGB"] = DC1394_VIDEO_MODE_1280x960_RGB8;
    _mapCameraModes["MODE_1280x960_MONO"] = DC1394_VIDEO_MODE_1280x960_MONO8;
    _mapCameraModes["MODE_1600x1200_YUV422"] = DC1394_VIDEO_MODE_1600x1200_YUV422;
    _mapCameraModes["MODE_1600x1200_RGB"] = DC1394_VIDEO_MODE_1600x1200_RGB8;
    _mapCameraModes["MODE_1600x1200_MONO"] = DC1394_VIDEO_MODE_1600x1200_MONO8;
    _mapCameraModes["MODE_1280x960_MONO16"] = DC1394_VIDEO_MODE_1280x960_MONO16;
    _mapCameraModes["MODE_1600x1200_MONO16"] = DC1394_VIDEO_MODE_1600x1200_MONO16;
    // format_7
    _mapCameraModes["MODE_FORMAT7_0"] = DC1394_VIDEO_MODE_FORMAT7_0;

    mapcv[DC1394_COLOR_CODING_MONO8] = pair<int,int>(IPL_DEPTH_8U,1);
    mapcv[DC1394_COLOR_CODING_YUV411] = pair<int,int>(IPL_DEPTH_8U,3);
    mapcv[DC1394_COLOR_CODING_YUV422] = pair<int,int>(IPL_DEPTH_8U,3);
    mapcv[DC1394_COLOR_CODING_YUV444] = pair<int,int>(IPL_DEPTH_8U,3);
    mapcv[DC1394_COLOR_CODING_RGB8] = pair<int,int>(IPL_DEPTH_8U,3);
    mapcv[DC1394_COLOR_CODING_MONO16] = pair<int,int>(IPL_DEPTH_16U,1);
    mapcv[DC1394_COLOR_CODING_RGB16] = pair<int,int>(IPL_DEPTH_16U,3);
    mapcv[DC1394_COLOR_CODING_MONO16S] = pair<int,int>(IPL_DEPTH_16S,1);
    mapcv[DC1394_COLOR_CODING_RGB16S] = pair<int,int>(IPL_DEPTH_16S,3);
    mapcv[DC1394_COLOR_CODING_RAW8] = pair<int,int>(IPL_DEPTH_8U,1);
    mapcv[DC1394_COLOR_CODING_RAW16] = pair<int,int>(IPL_DEPTH_16U,1);

    _node.param("display", display, 0);
    _node.param("mode",smode,string(""));
    map<string,dc1394video_mode_t>::iterator itmode = _mapCameraModes.find(smode);
    if( itmode != _mapCameraModes.end() )
      mode = itmode->second;
    else
      mode = (dc1394video_mode_t)0;

    _node.param("framerate",framerate,15.0);
    _node.param("square_roi",square_roi,0.0);

    // For ieee1394 cameras:
    _node.param("brightness",brightness,-1.0);
    _node.param("contrast",contrast,-1.0);
    _node.param("whitebalance_blueu",wb_blueu,-1.0);
    _node.param("whitebalance_redv",wb_redv,-1.0);
    _node.param("gain",gain,-1.0);
    _node.param("compression",compression,string(""));
    if( compression.size() == 0 )
      compression = "raw";
    _node.param("exposure",exposure,-1.0);
    _node.param("shutter",shutter,-1.0);

    string frame_id;
    _node.param("frame_id",frame_id,string(""));
    _undistorted_infomsg.header.frame_id = frame_id;

    // find the correct gui
    uint64_t guid;
    string sguid;
    if( _node.getParam("cameraguid",sguid) ) {
      // check if guid exists, if not, kill the node
      guid = (uint64_t)strtoull(sguid.c_str(),0,16);
      int i;
      for(i = 0; i < numcams; ++i) {
        if( guid == dcam::getGuid(i) )
          break;
      }

      ROS_ASSERT(i < numcams);
    }
    else {
      // pick any camera
      int cameraindex;
      _node.param("cameraindex",cameraindex,0);
      guid = dcam::getGuid(cameraindex % numcams);
    }

    string sbayer;
    if( _node.getParam("colorfilter",sbayer) ) {
      bEnableBayer = true;
      if( sbayer == "COLOR_FILTER_RGGB" )
        bayer = DC1394_COLOR_FILTER_RGGB;
      else if( sbayer == "COLOR_FILTER_GBRG" )
        bayer = DC1394_COLOR_FILTER_GBRG;
      else if( sbayer == "COLOR_FILTER_GRBG" )
        bayer = DC1394_COLOR_FILTER_GRBG;
      else if( sbayer == "COLOR_FILTER_BGGR" )
        bayer = DC1394_COLOR_FILTER_BGGR;
      else {
        ROS_ERROR("invalid colorfilter %s\n", sbayer.c_str());
        bEnableBayer = false;
      }
    }

    if( !StartCamera(guid) ) {
      fprintf(stderr, "couldn't open camera\n");
      return;
    }
  }

  virtual ~CameraOpenCVNode()
  {
    if( !!cam )
      cam->stop();

    if( frame != NULL )
      cvReleaseImage(&frame);
    if( frame_undist != NULL )
      cvReleaseImage(&frame_undist);
    if( _pUndistortionMapX != NULL )
      cvReleaseImage(&_pUndistortionMapX);
    if( _pUndistortionMapY != NULL )
      cvReleaseImage(&_pUndistortionMapY);
    dcam::fini();
  }

  bool StartCamera(uint64_t guid)
  {
    // set the real framerate, necessary because opencv doesn't set it correctly
    map<dc1394framerate_t,float> m;
    m[DC1394_FRAMERATE_1_875] = 1.875;
    m[DC1394_FRAMERATE_3_75] = 3.75;
    m[DC1394_FRAMERATE_7_5] = 7.5;
    m[DC1394_FRAMERATE_15] = 15;
    m[DC1394_FRAMERATE_30] = 30;
    m[DC1394_FRAMERATE_60] = 60;
    m[DC1394_FRAMERATE_120] = 120;
    m[DC1394_FRAMERATE_240] = 240;
    dc1394framerate_t fps = DC1394_FRAMERATE_30;
    for(map<dc1394framerate_t,float>::iterator it = m.begin(); it != m.end(); ++it) {
      if( fabsf(it->second-framerate) < 0.01f ) {
        fps = it->first;
        break;
      }
    }

    cambuffersize = 8;
    cam.reset(new NewDcam(guid, cambuffersize));
    if( mode == (dc1394video_mode_t)0 ) {
      if( cam->getModes() != NULL && cam->getModes()->num > 0 )
        mode = cam->getModes()->modes[0];
      else
        mode = DC1394_VIDEO_MODE_640x480_MONO8;
    }
    cam->setFormat(mode,fps,DC1394_ISO_SPEED_400);
    if( (mode == DC1394_VIDEO_MODE_FORMAT7_0) && square_roi ) {
      cam->setSquareROI(mode);
    }

    const dc1394feature_mode_t fmode[] = {DC1394_FEATURE_MODE_AUTO, DC1394_FEATURE_MODE_MANUAL};
    cam->setFeatureMode(DC1394_FEATURE_BRIGHTNESS, fmode[brightness >= 0]);
    if( brightness >= 0 )
      cam->setFeature(DC1394_FEATURE_BRIGHTNESS,brightness);

    // contrast is gamma
    cam->setFeatureMode(DC1394_FEATURE_GAMMA, fmode[contrast >= 0]);
    if( contrast >= 0 )
      cam->setFeature(DC1394_FEATURE_GAMMA,contrast);

    cam->setFeatureMode(DC1394_FEATURE_GAIN, fmode[gain >= 0]);
    if( gain >= 0 )
      cam->setFeature(DC1394_FEATURE_GAIN,gain);

    cam->setFeatureMode(DC1394_FEATURE_EXPOSURE, fmode[exposure >= 0]);
    if( exposure >= 0 )
      cam->setFeature(DC1394_FEATURE_EXPOSURE,exposure);

    cam->setFeatureMode(DC1394_FEATURE_SHUTTER,fmode[shutter >= 0]);
    if(shutter >= 0)
      cam->setFeature(DC1394_FEATURE_SHUTTER,shutter);

    cam->setFeatureMode(DC1394_FEATURE_WHITE_BALANCE, fmode[wb_blueu >= 0 && wb_redv >= 0]);
    if( wb_blueu >= 0 && wb_redv >= 0 ) {
      cam->setFeature(DC1394_FEATURE_WHITE_BALANCE,(unsigned int)wb_blueu, (unsigned int)wb_redv);
    }

    cam->start();
    ROS_INFO("camera started");
    camguid = guid;
    return true;
  }

  bool process()
  {
    if( !cam->getImage((int)(1000.0f/framerate)+100) ) {
      // camera might be bad, so restart
      cam.reset();
      if( !StartCamera(camguid) )
        fprintf(stderr, "couldn't open camera\n");
      return false;
    }

    ros::Time imagetime = ros::Time::now();
    dc1394video_frame_t* pframe = cam->getFrame();

    // convert to an opencv image
    if( frame == NULL ) {
      // initialize calibration params
      double KK_fx_original,KK_fy_original,KK_cx_original,KK_cy_original;
      _node.param("KK_fx_original",KK_fx_original,(double)pframe->size[0]);
      _node.param("KK_fy_original",KK_fy_original,(double)pframe->size[1]);
      _node.param("KK_cx_original",KK_cx_original,(double)pframe->size[0]/2.0);
      _node.param("KK_cy_original",KK_cy_original,(double)pframe->size[1]/2.0);
      double kc_k1_original,kc_k2_original,kc_p1_original,kc_p2_original;
      _node.param("kc_k1_original",kc_k1_original,0.0);
      _node.param("kc_k2_original",kc_k2_original,0.0);
      _node.param("kc_p1_original",kc_p1_original,0.0);
      _node.param("kc_p2_original",kc_p2_original,0.0);
      kc_original[0] = kc_k1_original; kc_original[1] = kc_k2_original; kc_original[2] = kc_p1_original; kc_original[3] = kc_p2_original; kc_original[4] = 0;
      double KK_fx_undistorted,KK_fy_undistorted,KK_cx_undistorted,KK_cy_undistorted;
      _node.param("KK_fx_undistorted",KK_fx_undistorted,(double)pframe->size[0]);
      _node.param("KK_fy_undistorted",KK_fy_undistorted,(double)pframe->size[1]);
      _node.param("KK_cx_undistorted",KK_cx_undistorted,(double)pframe->size[0]/2.0);
      _node.param("KK_cy_undistorted",KK_cy_undistorted,(double)pframe->size[1]/2.0);
      double kc_k1_undistorted,kc_k2_undistorted,kc_p1_undistorted,kc_p2_undistorted;
      _node.param("kc_k1_undistorted",kc_k1_undistorted,0.0);
      _node.param("kc_k2_undistorted",kc_k2_undistorted,0.0);
      _node.param("kc_p1_undistorted",kc_p1_undistorted,0.0);
      _node.param("kc_p2_undistorted",kc_p2_undistorted,0.0);
      kc_undistorted[0] = kc_k1_undistorted; kc_undistorted[1] = kc_k2_undistorted; kc_undistorted[2] = kc_p1_undistorted; kc_undistorted[3] = kc_p2_undistorted; kc_undistorted[4] = 0;

      // ROI
      // _node.param("roi_undistorted_x_offset",roi_undistorted_x_offset,0.0);
      // _node.param("roi_undistorted_y_offset",roi_undistorted_y_offset,0.0);
      // _node.param("roi_undistorted_height",roi_undistorted_height,(double)pframe->size[1]);
      // _node.param("roi_undistorted_width",roi_undistorted_width,(double)pframe->size[0]);

      // create image
      pair<int,int> coding = mapcv[pframe->color_coding];
      if (square_roi) {
        if( bEnableBayer )
          frame = cvCreateImage( cvSize(pframe->size[1], pframe->size[1]), coding.first, 3);
        else
          frame = cvCreateImage( cvSize(pframe->size[1], pframe->size[1]), coding.first, coding.second);
        frame_undist = cvCloneImage(frame);
      }
      else {
        if( bEnableBayer )
          frame = cvCreateImage( cvSize(pframe->size[0], pframe->size[1]), coding.first, 3);
        else
          frame = cvCreateImage( cvSize(pframe->size[0], pframe->size[1]), coding.first, coding.second);
        frame_undist = cvCloneImage(frame);
      }


      if( display ) {
        stringstream ss;
        ss << "opencv camera: " << frame->width << "x" << frame->height << "  fps: " << framerate;
        windowname = ss.str();
        cvNamedWindow(windowname.c_str(), CV_WINDOW_AUTOSIZE);
        cvSetMouseCallback(windowname.c_str(), MouseCallback, this);
        cvStartWindowThread();
      }

      _undistorted_infomsg.width = frame->width;
      _undistorted_infomsg.height = frame->height;
      // _undistorted_infomsg.roi.x_offset = roi_undistorted_x_offset;
      // _undistorted_infomsg.roi.y_offset = roi_undistorted_y_offset;
      // _undistorted_infomsg.roi.height = roi_undistorted_height;
      // _undistorted_infomsg.roi.width = roi_undistorted_width;
      for(int i = 0; i < 5; ++i)
        _undistorted_infomsg.D[i] = kc_undistorted[i];
      _undistorted_infomsg.K[0] = KK_fx_undistorted; _undistorted_infomsg.K[1] = 0; _undistorted_infomsg.K[2] = KK_cx_undistorted;
      _undistorted_infomsg.K[3] = 0; _undistorted_infomsg.K[4] = KK_fy_undistorted; _undistorted_infomsg.K[5] = KK_cy_undistorted;
      _undistorted_infomsg.K[6] = 0; _undistorted_infomsg.K[7] = 0; _undistorted_infomsg.K[8] = 1;
      _undistorted_infomsg.R[0] = 1; _undistorted_infomsg.R[1] = 0; _undistorted_infomsg.R[2] = 0;
      _undistorted_infomsg.R[3] = 0; _undistorted_infomsg.R[4] = 1; _undistorted_infomsg.R[5] = 0;
      _undistorted_infomsg.R[6] = 0; _undistorted_infomsg.R[7] = 0; _undistorted_infomsg.R[8] = 1;
      for(int i = 0; i < 3; ++i) {
        _undistorted_infomsg.P[4*i+0] = _undistorted_infomsg.K[3*i+0];
        _undistorted_infomsg.P[4*i+1] = _undistorted_infomsg.K[3*i+1];
        _undistorted_infomsg.P[4*i+2] = _undistorted_infomsg.K[3*i+2];
        _undistorted_infomsg.P[4*i+3] = 0;
      }

      _original_infomsg.width = frame->width;
      _original_infomsg.height = frame->height;
      for(int i = 0; i < 5; ++i)
        _original_infomsg.D[i] = kc_original[i];
      _original_infomsg.K[0] = KK_fx_original; _original_infomsg.K[1] = 0; _original_infomsg.K[2] = KK_cx_original;
      _original_infomsg.K[3] = 0; _original_infomsg.K[4] = KK_fy_original; _original_infomsg.K[5] = KK_cy_original;
      _original_infomsg.K[6] = 0; _original_infomsg.K[7] = 0; _original_infomsg.K[8] = 1;
      _original_infomsg.R[0] = 1; _original_infomsg.R[1] = 0; _original_infomsg.R[2] = 0;
      _original_infomsg.R[3] = 0; _original_infomsg.R[4] = 1; _original_infomsg.R[5] = 0;
      _original_infomsg.R[6] = 0; _original_infomsg.R[7] = 0; _original_infomsg.R[8] = 1;
      for(int i = 0; i < 3; ++i) {
        _original_infomsg.P[4*i+0] = _original_infomsg.K[3*i+0];
        _original_infomsg.P[4*i+1] = _original_infomsg.K[3*i+1];
        _original_infomsg.P[4*i+2] = _original_infomsg.K[3*i+2];
        _original_infomsg.P[4*i+3] = 0;
      }

      if( _pUndistortionMapY == NULL ) {
        _pUndistortionMapX = cvCreateImage( cvSize(pframe->size[0], pframe->size[1]), IPL_DEPTH_32F, 1);
        _pUndistortionMapY = cvCreateImage( cvSize(pframe->size[0], pframe->size[1]), IPL_DEPTH_32F, 1);

        // Fill the maps : intrinMat is a float[9] intrinsic
        // coeff buffer and distortion a float[4] buffer
        CvMat imat = cvMat( 3, 3, sizeof(_undistorted_infomsg.K[0]) == 4 ? CV_32FC1 : CV_64FC1, &_undistorted_infomsg.K[0] );
        CvMat dist = cvMat( 4, 1, sizeof(kc_original[0]) == 4 ? CV_32FC1 : CV_64FC1, kc_original );

#ifdef HAVE_CV_UNDISTORT_RECTIFY_MAP // for newer opencv versions, *have* to use cvInitUndistortRectifyMap
        float feye[9] = {1,0,0,0,1,0,0,0,1};
        CvMat eye = cvMat(3,3,CV_32FC1, feye);
        cvInitUndistortRectifyMap(&imat,&dist,&eye,&imat,_pUndistortionMapX, _pUndistortionMapY);
#else
        cvInitUndistortMap( &imat, &dist, _pUndistortionMapX, _pUndistortionMapY );
#endif
      }
    }

    _undistorted_infomsg.header.stamp = imagetime;
    _original_infomsg.header = _undistorted_infomsg.header;
    _pubUndistortedInfo.publish(_undistorted_infomsg);
    _pubOriginalInfo.publish(_original_infomsg);

    bool bImagePublish = _pubUndistortedImage.getNumSubscribers()>0;
    bool bOriginalImagePublish = _pubOriginalImage.getNumSubscribers()>0;

    if( !bImagePublish && !bOriginalImagePublish && !display )
      return true; // exit before processing anything

    unsigned char * src = (unsigned char *)pframe->image;
    unsigned char * dst = (unsigned char *)frame->imageData;

    switch (pframe->color_coding) {
    case DC1394_COLOR_CODING_RGB8:
      // Convert RGB to BGR
      for (int i=0;i<frame->imageSize;i+=6) {
        dst[i]   = src[i+2];
        dst[i+1] = src[i+1];
        dst[i+2] = src[i];
        dst[i+3] = src[i+5];
        dst[i+4] = src[i+4];
        dst[i+5] = src[i+3];
      }
      break;
    case DC1394_COLOR_CODING_RGB16: {
      uint16_t* src16 = (uint16_t*)pframe->image;
      uint16_t* dst16 = (uint16_t*)frame->imageData;
      for (int i=0;i<frame->imageSize;i+=6) {
        dst16[i]   = src16[i+2];
        dst16[i+1] = src16[i+1];
        dst16[i+2] = src16[i];
        dst16[i+3] = src16[i+5];
        dst16[i+4] = src16[i+4];
        dst16[i+5] = src16[i+3];
      }
      break;
    }
    case DC1394_COLOR_CODING_YUV422:
      //printf("icvRetrieveFrame convert YUV422 to BGR %d\n");
      uyvy2bgr(src, dst, frame->width * frame->height);
      break;
    case DC1394_COLOR_CODING_MONO8:
      if( bEnableBayer ) {
        ;
        // dc1394error_t err = dc1394_bayer_decoding_8bit(src,dst,frame->width,frame->height,bayer,DC1394_BAYER_METHOD_BILINEAR);
        // if( err != DC1394_SUCCESS )
        //   ROS_WARN("bayer decoding error: %s",dc1394_error_get_string(err));
      }
      else
        memcpy(dst,src,frame->width*frame->height);
      break;
    case DC1394_COLOR_CODING_MONO16:
      memcpy(dst,src,frame->width*frame->height*2);
      break;
    case DC1394_COLOR_CODING_YUV411:
      //printf("icvRetrieveFrame convert YUV411 to BGR %d\n");
      uyyvyy2bgr(src, dst, frame->width * frame->height);
      break;
    case DC1394_COLOR_CODING_YUV444:
      //printf("icvRetrieveFrame convert YUV444 to BGR %d\n");
      uyv2bgr(src, dst, frame->width * frame->height);
      break;
    default:
      fprintf(stderr,"%s:%d: Unsupported color mode %d\n",__FILE__,__LINE__,pframe->color_coding);
      ROS_BREAK();
    }

    if( bOriginalImagePublish ) {
      if (sensor_msgs::CvBridge::fromIpltoRosImage(frame, _imagemsg, "passthrough")) {
        _imagemsg.header = _original_infomsg.header;
        _pubOriginalImage.publish(_imagemsg);
      }
      else
        ROS_ERROR("error publishing orignal image");
    }

    if( bImagePublish || display) {
      cvRemap( frame, frame_undist, _pUndistortionMapX, _pUndistortionMapY, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
      // cvSetImageROI( frame_undist, cvRect(int(roi_undistorted_x_offset),int(roi_undistorted_y_offset),int(roi_undistorted_width),int(roi_undistorted_height)));
    }

    if( bImagePublish ) {
      if (sensor_msgs::CvBridge::fromIpltoRosImage(frame_undist, _imagemsg, "passthrough")) {
        _imagemsg.header = _undistorted_infomsg.header;
        _pubUndistortedImage.publish(_imagemsg);
      }
      else
        ROS_ERROR("error publishing undistorted image");
    }

    if (display) {
      if( bSnapImage ) {
        string imfile = get_available_filename("image%03d.png");
        cvSaveImage(imfile.c_str(), frame_undist);
        fprintf(stderr, "saving undistorted image %s\n", imfile.c_str());
        bSnapImage = false;
      }

      cvShowImage(windowname.c_str(), frame_undist);
    }

    // cvResetImageROI(frame_undist);
    //usleep(max(1000,1000000/(int)framerate-10000));
    return true;
  }

  bool ok() { return _node.ok(); }

private:

  static void MouseCallback(int event, int x, int y, int flags, void* param)
  {
    ((CameraOpenCVNode*)param)->_MouseCallback(event, x, y, flags);
  }

  void _MouseCallback(int event, int x, int y, int flags)
  {
    switch(event){
    case CV_EVENT_MBUTTONDOWN:
      bSnapImage = true;
      break;
    case CV_EVENT_RBUTTONDOWN:
      bToggleVideoWriter = true;
      break;
    }
  }

  string get_available_filename(const string &pat)
  {
    for(int num=0; ; ++num) {
      char fname[PATH_MAX];
      sprintf(fname,pat.c_str(), num);
      struct stat statbuf;
      int ret = stat(fname,&statbuf);
      if(ret==-1 && errno==ENOENT)
        return string(fname);
      if(S_ISREG(statbuf.st_mode))
        continue;
      else {
        perror("error: trouble scanning directory");
        return string();
      }
    }
    return string();
  }

  /**********************************************************************
   *
   *  CONVERSION FUNCTIONS TO RGB 24bpp
   *
   **********************************************************************/
  static void uyv2bgr(const unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
  {
    register int i = NumPixels + (NumPixels << 1) - 1;
    register int j = NumPixels + (NumPixels << 1) - 1;
    register int y, u, v;
    register int r, g, b;

    while (i > 0) {
      v = src[i--] - 128;
      y = src[i--];
      u = src[i--] - 128;
      YUV2RGB(y, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
    }
  }

  static void uyvy2bgr(const unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
  {
    register int i = (NumPixels << 1) - 1;
    register int j = NumPixels + (NumPixels << 1) - 1;
    register int y0, y1, u, v;
    register int r, g, b;

    while (i > 0) {
      y1 = src[i--];
      v = src[i--] - 128;
      y0 = src[i--];
      u = src[i--] - 128;
      YUV2RGB(y1, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
      YUV2RGB(y0, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
    }
  }


  static void uyyvyy2bgr(const unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
  {
    register int i = NumPixels + (NumPixels >> 1) - 1;
    register int j = NumPixels + (NumPixels << 1) - 1;
    register int y0, y1, y2, y3, u, v;
    register int r, g, b;

    while (i > 0) {
      y3 = src[i--];
      y2 = src[i--];
      v = src[i--] - 128;
      y1 = src[i--];
      y0 = src[i--];
      u = src[i--] - 128;
      YUV2RGB(y3, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
      YUV2RGB(y2, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
      YUV2RGB(y1, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
      YUV2RGB(y0, u, v, r, g, b);
      dest[j--] = r;
      dest[j--] = g;
      dest[j--] = b;
    }
  }

  // this one was in coriander but didn't take bits into account
  static void rgb482bgr(const unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits)
  {
    register int i = (NumPixels << 1) - 1;
    register int j = NumPixels + (NumPixels << 1) - 1;
    register int y;

    while (i > 0) {
      y = src[i--];
      dest[j-2] = (y + (src[i--] << 8)) >> (bits - 8);
      j--;
      y = src[i--];
      dest[j] = (y + (src[i--] << 8)) >> (bits - 8);
      j--;
      y = src[i--];
      dest[j+2] = (y + (src[i--] << 8)) >> (bits - 8);
      j--;
    }
  }

  bool bToggleVideoWriter, bSnapImage;
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"CameraFirewire");

  if( !ros::master::check() )
    return 1;

  CameraOpenCVNode cvcam;

  while (cvcam.ok()) {
    if (!cvcam.process())
      usleep(100000);
  }

  return 0;
}
