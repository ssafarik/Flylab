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
#include <cstdio>
#include <vector>
#include <sstream>

#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>

#include <cv_bridge/CvBridge.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;
using namespace ros;

class ImageScalerNode
{
public:
  class PyramidImage
  {
  public:
    PyramidImage() : pimage(NULL) {}
    virtual ~PyramidImage() {
      if( pimage != NULL )
        cvReleaseImage(&pimage);
    }

    string topic;
    IplImage* pimage;
    sensor_msgs::Image imagemsg;
    sensor_msgs::CameraInfo infomsg;
    Publisher _pubInfo;
    image_transport::Publisher _pubImage;
  };

  boost::mutex _mutex;
  ros::NodeHandle _node;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _sub;
  Subscriber _subInfo;

  int nPyramidLevels;
  vector<boost::shared_ptr<PyramidImage> > vpyramid;
  sensor_msgs::CvBridge _bridge;
  sensor_msgs::CameraInfo _camerainfo;

  ImageScalerNode() : _it(_node)
  {
    _node.param("pyramidlevels",nPyramidLevels,1);
    if( nPyramidLevels <= 0 )
      throw string("pyramidlevels needs to be greater than 0");

    _sub = _it.subscribe("Image",1,&ImageScalerNode::image_cb,this);
    _subInfo = _node.subscribe("CameraInfo",1,&ImageScalerNode::info_cb,this);

    vpyramid.resize(nPyramidLevels);
    for(int i = 0; i < nPyramidLevels; ++i) {
      vpyramid[i].reset(new PyramidImage());
      stringstream ss;
      ss << "CameraInfo" << (2<<i) << "d";
      vpyramid[i]->topic = ss.str();
      vpyramid[i]->_pubInfo = _node.advertise<sensor_msgs::CameraInfo>(ss.str(),2);

      ss.str("");
      ss << "Image" << (2<<i) << "d";
      vpyramid[i]->_pubImage = _it.advertise(ss.str(),2);
    }
  }
  ~ImageScalerNode() {}

  void info_cb(const sensor_msgs::CameraInfoConstPtr& msg_ptr)
  {
    boost::mutex::scoped_lock lock(_mutex);
    _camerainfo = *msg_ptr;
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr)
  {

    IplImage *frame = NULL;
    try{
      try {
        frame = _bridge.imgMsgToCv(msg_ptr, "bgr8");
      }
      catch (const cv::Exception& err) {
        frame = _bridge.imgMsgToCv(msg_ptr, "passthrough");
      }
    }
    catch (const sensor_msgs::CvBridgeException& error) {
      ROS_WARN("bad frame");
      return;
    }

    for(int i = 0; i < nPyramidLevels; ++i) {
      if( vpyramid[i]->pimage != NULL && (vpyramid[i]->pimage->width!=frame->width/2 || vpyramid[i]->pimage->height!=frame->height/2) ) {
        cvReleaseImage(&vpyramid[i]->pimage);
      }
      if( vpyramid[i]->pimage == NULL )
        vpyramid[i]->pimage = cvCreateImage(cvSize(frame->width/2,frame->height/2),frame->depth,frame->nChannels);
      cvPyrDown(frame,vpyramid[i]->pimage);

      // fill the msg
      if (sensor_msgs::CvBridge::fromIpltoRosImage(vpyramid[i]->pimage, vpyramid[i]->imagemsg, "passthrough")) {
        vpyramid[i]->imagemsg.header = msg_ptr->header;
        vpyramid[i]->_pubImage.publish(vpyramid[i]->imagemsg);
      }
      else
        ROS_ERROR("error publishing scaled image");

      frame = vpyramid[i]->pimage;
    }

    {
      boost::mutex::scoped_lock lock(_mutex);
      for(int i = 0; i < nPyramidLevels; ++i) {
        float fScale = 1.0f/(float)(2<<i);
        vpyramid[i]->infomsg = _camerainfo;
        for(int j = 0; j < 9; ++j)
          vpyramid[i]->infomsg.K[j] = _camerainfo.K[j]*(j<6?fScale:1.0f);
        vpyramid[i]->infomsg.width /= 2<<i;
        vpyramid[i]->infomsg.height /= 2<<i;
        vpyramid[i]->infomsg.header = msg_ptr->header;
        vpyramid[i]->_pubInfo.publish(vpyramid[i]->infomsg);
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"imagescaler");

  if( !ros::master::check() )
    return 1;

  boost::shared_ptr<ImageScalerNode> scalernode(new ImageScalerNode());

  ros::spin();
  scalernode.reset();
  return 0;
}
