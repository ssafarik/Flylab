/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//
// Basic driver for grabbing frames and controlling IIDC 1.3x cameras
// Uses libdc1394 under Linux
// Uses CMU driver under MSW
//

#include "dcam1394/dcam1394.h"
#include <cstring>
#include <cstdio>
#include <errno.h>

#define PRINTF(a...) ROS_INFO(a)

#define CHECK_READY()                                                   \
  if (!dcam::dcRef) {                                                   \
    char msg[256];                                                      \
    snprintf(msg, 256, "Tried to call %s before calling dcam::init()", __FUNCTION__); \
    throw DcamException(msg);                                           \
  }

#define CHECK_ERR(fnc, amsg)                                            \
  {                                                                     \
    dc1394error_t err = fnc;                                            \
    if (err != DC1394_SUCCESS) {                                        \
      char msg[256];                                                    \
      snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg); \
      throw DcamException(msg);                                         \
    }                                                                   \
  }

#define CHECK_ERR_CLEAN(fnc, amsg)                                      \
  {                                                                     \
    dc1394error_t err = fnc;                                            \
    if (err != DC1394_SUCCESS) {                                        \
      cleanup();                                                        \
      char msg[256];                                                    \
      snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg); \
      throw DcamException(msg);                                         \
    }                                                                   \
  }


dc1394_t *dcam::dcRef = NULL;   // system object, dc1394_t* for Linux

void
dcam::init()
{
  if (dcam::dcRef == NULL)
    {
      dcam::dcRef = dc1394_new();

      if (dcam::dcRef == NULL)
        {
          throw DcamException("Could not initialize dc1394_context.  Make sure /dev/raw1394 exists and you have permissions to access.");
        }

      if (numCameras() > 0)
        {
          dc1394camera_t *camera = dc1394_camera_new((dc1394_t *)dcam::dcRef, getGuid(0));
          if (!camera) {
            char msg[256];
            snprintf(msg, 256, "Could not acquire camera to reset bus in %s", __FUNCTION__);
            throw DcamException(msg);
          }

          PRINTF("Reset bus");
          dc1394_reset_bus(camera);

          usleep(500000);

          PRINTF("Initializing camera, turning off ISO");
          dc1394_video_set_transmission(camera, DC1394_OFF);

          dc1394_camera_free(camera);
          dc1394_free((dc1394_t *)dcam::dcRef);

          dcam::dcRef = dc1394_new();
        }

      if (dcam::dcRef == NULL)
        {
          throw DcamException("Could not initialize dc1394_context.  Make sure /dev/raw1394 exists and you have permissions to access.");
        }

      usleep(500000);

    }
}

void
dcam::fini()
{
  if (dcam::dcRef != NULL)
    {
      dc1394_free((dc1394_t*)dcam::dcRef);
    }
}


size_t dcam::numCameras()
{
  CHECK_READY();

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dcam::dcRef, &list), "Could not enumerate cameras" );

  size_t num = list->num;

  dc1394_camera_free_list (list);

  return num;
}


uint64_t
dcam::getGuid(size_t i)
{
  if (!dcam::dcRef) {
    char msg[256];
    snprintf(msg, 256, "Tried to call %s before calling dcam::init()", __FUNCTION__);
    throw DcamException(msg);
  }

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dcam::dcRef, &list), "Could not enumerate cameras" );

  if (i >= list->num)
    throw DcamException("Tried to get Guid of non-existant camera");

  uint64_t guid = list->ids[i].guid;

  dc1394_camera_free_list (list);

  return guid;
}


// Model name

char *
dcam::getModel(size_t i)
{
  char *name = NULL;
  CHECK_READY();

  if (i < numCameras())
    {
      dc1394camera_t *camera = dc1394_camera_new(dcam::dcRef, getGuid(i));
      if (!camera)
        {
          char msg[256];
          snprintf(msg, 256, "Could not acquire camera %d %s", int(i), __FUNCTION__);
          throw DcamException(msg);
        }

      if (camera->model)
        name = strdup(camera->model);

      dc1394_camera_free(camera);
    }

  return name;
}


// Vendor name

char *
dcam::getVendor(size_t i)
{
  char *name = NULL;
  CHECK_READY();

  if (i < numCameras())
    {
      dc1394camera_t *camera = dc1394_camera_new(dcam::dcRef, getGuid(i));
      if (!camera)
        {
          char msg[256];
          snprintf(msg, 256, "Could not acquire camera %d %s", int(i), __FUNCTION__);
          throw DcamException(msg);
        }

      if (camera->vendor)
        name = strdup(camera->vendor);
      dc1394_camera_free(camera);
    }

  return name;
}


static const char *modestrings[DC1394_VIDEO_MODE_NUM] =
  {
    "DC1394_VIDEO_MODE_160x120_YUV444",
    "DC1394_VIDEO_MODE_320x240_YUV422",
    "DC1394_VIDEO_MODE_640x480_YUV411",
    "DC1394_VIDEO_MODE_640x480_YUV422",
    "DC1394_VIDEO_MODE_640x480_RGB8",
    "DC1394_VIDEO_MODE_640x480_MONO8",
    "DC1394_VIDEO_MODE_640x480_MONO16",
    "DC1394_VIDEO_MODE_800x600_YUV422",
    "DC1394_VIDEO_MODE_800x600_RGB8",
    "DC1394_VIDEO_MODE_800x600_MONO8",
    "DC1394_VIDEO_MODE_1024x768_YUV422",
    "DC1394_VIDEO_MODE_1024x768_RGB8",
    "DC1394_VIDEO_MODE_1024x768_MONO8",
    "DC1394_VIDEO_MODE_800x600_MONO16",
    "DC1394_VIDEO_MODE_1024x768_MONO16",
    "DC1394_VIDEO_MODE_1280x960_YUV422",
    "DC1394_VIDEO_MODE_1280x960_RGB8",
    "DC1394_VIDEO_MODE_1280x960_MONO8",
    "DC1394_VIDEO_MODE_1600x1200_YUV422",
    "DC1394_VIDEO_MODE_1600x1200_RGB8",
    "DC1394_VIDEO_MODE_1600x1200_MONO8",
    "DC1394_VIDEO_MODE_1280x960_MONO16",
    "DC1394_VIDEO_MODE_1600x1200_MONO16",
    "DC1394_VIDEO_MODE_EXIF",
    "DC1394_VIDEO_MODE_FORMAT7_0",
    "DC1394_VIDEO_MODE_FORMAT7_1",
    "DC1394_VIDEO_MODE_FORMAT7_2",
    "DC1394_VIDEO_MODE_FORMAT7_3",
    "DC1394_VIDEO_MODE_FORMAT7_4",
    "DC1394_VIDEO_MODE_FORMAT7_5",
    "DC1394_VIDEO_MODE_FORMAT7_6",
    "DC1394_VIDEO_MODE_FORMAT7_7"
  };

// mode strings from mode
const char *
dcam::getModeString(dc1394video_mode_t mode)
{
  if (mode < DC1394_VIDEO_MODE_MAX)
    return modestrings[mode-DC1394_VIDEO_MODE_MIN];
  else
    return "";
}


// Set up a camera object

dcam::Dcam::Dcam(uint64_t guid, size_t bsize)
{
  CHECK_READY();
  dcCam = dc1394_camera_new(dcRef, guid);

  if (!dcCam)
    throw DcamException("Could not create camera");

  CHECK_ERR( dc1394_video_get_supported_modes(dcCam, &camModes),
             "Could not get supported modes" );

  bufferSize = bsize;
  camPolicy = DC1394_CAPTURE_POLICY_POLL;
  camFrame = NULL;
  camIm = new ImageData();
  camIm->params = NULL;
  camIm->imRaw = NULL;
  camIm->im = NULL;
  camIm->imColor = NULL;
  camIm->imRect = NULL;
  camIm->imRectColor = NULL;
  camIm->imWidth = 0;
  camIm->imHeight = 0;
  isSTOC = false;
  isVidereStereo = false;
  isVidere = false;
  isColor = false;
  procMode = PROC_MODE_RECTIFIED;

  //  dc1394_camera_print_info(dcCam,stdout);

  // Check Videre camera type and local params
  if (!strcmp(getModel(),"MDS-STH")) // Videre-type camera
    {
      isVidere = true;

      PRINTF("[dcam] Videre camera, getting local params");
      uint32_t qval;

      // firmware level
      qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_FW_LEVEL_OFFSET);
      int major = (qval & 0x0000ff00)>>8;
      int minor = (qval & 0x000000ff);

      if ((qval >> 16) != 0 || major < 2 || minor > 10) // check for local parameters
        PRINTF("[dcam] No local parameters");
      else
        {
          // Camera and imager firmware
          camFirmware = qval & 0xffff;
          PRINTF("[dcam] Camera firmware: %02d.%02d", major, minor);
          qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_LEVEL_OFFSET);
          imFirmware = qval & 0xff;
          PRINTF("[dcam] Imager firmware: %04x", imFirmware);
          if ((qval & 0xff0000)==0x080000)
            {
              isVidereStereo = true;
              PRINTF("[Dcam] Found stereo device");
            }

          // STOC
          qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_PROC_OFFSET);
          stocFirmware = (qval & 0xffff00) >> 8;
          major = (stocFirmware & 0xff00)>>8;
          minor = stocFirmware & 0xff;
          PRINTF("[dcam] STOC version: %02d.%02d", major, minor);

          if (major > 0 && major < 0xff && minor != 0xff) // check for odd firmware values
            {
              isSTOC = true;
              procMode = (videre_proc_mode_t)(qval & 0x000000ff);

              // this sets the Config bits on faulty FPGA firmware (version 4.1 and below)
              qval = 0x08000000 | (0x9C << 16);
              setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
            }

          // STOC thresholds
          qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_PROC_THRESH_OFFSET);
          PRINTF("[dcam] STOC thresholds: %08x", qval);

          // parameter string
          if (getParameters() != NULL)
            PRINTF("[dcam] Calibration, %d bytes", strlen(camIm->params));
          else
            PRINTF("[dcam] No calibration");

        }
    }

  // check for color/monochrome camera
  isColor = false;
  if (hasFeature(DC1394_FEATURE_WHITE_BALANCE))
    {
      isColor = true;
      PRINTF("[dcam] Color device");
    }
  else
    PRINTF("[dcam] Monochrome device");

  setRawType();

  // check registers
  uint32_t qval;
  qval = getRegister(0x404);
  PRINTF("Feature register hi: %08x", qval);
  qval = getRegister(0x408);
  PRINTF("Feature register lo: %08x", qval);

  // set up max/min values
  // NOTE: on Videre cameras with FW firmware < 6.0, control reg
  //   does not have presence switch on
  //

  expMin = expMax = 0;
  gainMin = gainMax = 0;
  brightMin = brightMax = 0;

  if (isVidere && camFirmware < 0x0600)
    {
      expMax = 530;
      gainMax = 48;
      brightMax = 255;

      uint32_t qval;
      qval = getRegister(0x504); // exposure inquiry reg
      expMax = qval & 0x3ff;
      PRINTF("[Dcam] Exposure min/max: [%d,%d]",expMin,expMax);

      qval = getRegister(0x520); // exposure inquiry reg
      gainMax = qval & 0x3ff;
      PRINTF("[Dcam] Gain min/max: [%d,%d]",gainMin,gainMax);

      qval = getRegister(0x500); // brightness inquiry reg
      brightMax = qval & 0x3ff;
      PRINTF("[Dcam] Brightness min/max: [%d,%d]",brightMin,brightMax);
    }

  else
    {
      if (hasFeature(DC1394_FEATURE_EXPOSURE))
        {
          getFeatureBoundaries(DC1394_FEATURE_EXPOSURE,expMin,expMax);
          PRINTF("[Dcam] Exposure min/max: [%d,%d]",expMin,expMax);
        }
      else
        PRINTF("[Dcam] No exposure feature");

      if (hasFeature(DC1394_FEATURE_GAIN))
        {
          getFeatureBoundaries(DC1394_FEATURE_GAIN,gainMin,gainMax);
          PRINTF("[Dcam] Gain min/max: [%d,%d]",gainMin,gainMax);
        }
      else
        PRINTF("[Dcam] No gain feature");

      if (hasFeature(DC1394_FEATURE_BRIGHTNESS))
        {
          getFeatureBoundaries(DC1394_FEATURE_BRIGHTNESS,brightMin,brightMax);
          PRINTF("[Dcam] Brightness min/max: [%d,%d]",brightMin,brightMax);
        }
      else
        PRINTF("[Dcam] No brightness feature");
    }

  //  dc1394_iso_release_bandwidth(dcCam, 10000000);

  //  CHECK_ERR_CLEAN( dc1394_reset_bus(dcCam), "Could not reset bus" );

}


// Tear down camera object

dcam::Dcam::~Dcam()
{
  if (dcCam != NULL)
    cleanup();
  free(camIm);
}

void
dcam::Dcam::cleanup()
{
  dc1394_video_set_transmission(dcCam, DC1394_OFF);
  dc1394_capture_stop(dcCam);
  dc1394_camera_free(dcCam);
  dcCam = NULL;
}



// Return a list of modes

dc1394video_modes_t *
dcam::Dcam::getModes()
{
  CHECK_READY();
  return &camModes;
}


// Model name

char *
dcam::Dcam::Dcam::getModel()
{
  return dcCam->model;
}


// Vendor name

char *
dcam::Dcam::getVendor()
{
  return dcCam->vendor;
}


// Set up image format

void
dcam::Dcam::setFormat(dc1394video_mode_t video,
                      dc1394framerate_t fps,
                      dc1394speed_t speed)
{
  videoMode = video;

  dc1394_capture_stop(dcCam);   // tear down any previous capture setup

  // check for valid video mode
  size_t i;
  for (i=0; i<camModes.num; i++)
    {
      if (camModes.modes[i] == video)
        break;
    }

  if (i >= camModes.num)        // oops, haven't found it
    {
      char msg[256];
      snprintf(msg, 256, "setFormat: not a valid mode: %s",
               getModeString(video));
      throw DcamException(msg);
    }

  CHECK_ERR_CLEAN( dc1394_video_set_mode(dcCam, video),
                   "Could not set video mode");
  CHECK_ERR_CLEAN( dc1394_video_set_iso_speed(dcCam, speed),
                   "Could not set iso speed");
  CHECK_ERR_CLEAN( dc1394_video_set_framerate(dcCam, fps),
                   "Could not set framerate");
  CHECK_ERR_CLEAN( dc1394_capture_setup(dcCam, bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT),
                   "Could not setup camera.");
  setRawType();
}

// Format7 functions

void
dcam::Dcam::setSquareROI(dc1394video_mode_t video)
{
  CHECK_READY();

  CHECK_ERR_CLEAN( dc1394_format7_get_max_image_size(dcCam, video, &hsize, &vsize),
                   "Could not find format7 max image size");

  CHECK_ERR_CLEAN( dc1394_format7_set_roi(dcCam, video, DC1394_COLOR_CODING_MONO8, DC1394_USE_MAX_AVAIL, (hsize-vsize)/2, (vsize-vsize), vsize, vsize),
                   "Could not set format7 image roi");

  // CHECK_ERR_CLEAN( dc1394_format7_set_image_size(dcCam, video, vsize, vsize),
  //                  "Could not set format7 image size");
  // CHECK_ERR_CLEAN( dc1394_format7_set_image_position(dcCam, video, (hsize-vsize)/2, (vsize-vsize)),
  //                  "Could not set format7 image position");

  // CHECK_ERR_CLEAN( dc1394_format7_get_packet_size(dcCam, video, &packet_size),
  //                  "Could not get format7 packet size");

  // fprintf(stderr, "hsize = %d vsize = %d\n",hsize,vsize);
  // fprintf(stderr, "packet_size = %d\n",packet_size);
}


// Start and stop streaming

void
dcam::Dcam::start()
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_ON),
                   "Could not start camera iso transmission");


  // Some camera take a little while to start.  We check 10 times over the course of a second:
  int tries = 10;

  while (tries-- > 0)
    {
      // now check if we have started transmission, no error from set_transmission
      dc1394switch_t pwr;
      dc1394_video_get_transmission(dcCam, &pwr);
      if (pwr == DC1394_ON)
        {
          started = true;
          return;
        }
      usleep(10000);
    }

  throw DcamException("Camera iso transmission did not actually start.");
}



void
dcam::Dcam::stop()
{
  if (camFrame)
    CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, camFrame), "Could not release frame");

  camFrame = NULL;
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_OFF),
                   "Could not stop camera iso transmission");

  started = false;
}


// Getting images
// Waits for the next image available, up to ms for timeout
//   Assumes capture policy of POLL
// Stores the next available image into the class instance

bool
dcam::Dcam::getImage(int ms)
{
  CHECK_READY();

  if (!started)
    return false;

  // release previous frame, if it exists
  if (camFrame)
    CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, camFrame),
                     "Could not release frame");

  camFrame = NULL;

  // get the image
  while (1)
    {
      CHECK_ERR_CLEAN( dc1394_capture_dequeue(dcCam, camPolicy, &camFrame),
                       "Could not capture frame");

      if (camFrame == NULL)
        {
          if (ms <= 0) break;
          ms -= 10;
          usleep(10000);
        }
      else
        {
          //        break;
          while (1)             // flush the buffer, get latest one
            {
              dc1394video_frame_t *f = NULL;
              dc1394_capture_dequeue(dcCam, camPolicy, &f);
              if (f != NULL)
                {
                  dc1394_capture_enqueue(dcCam,camFrame);
                  camFrame = f;
                }
              else
                break;
            }
          break;
        }
    }

  // transfer info
  if (camFrame)
    {
      // clear everything out first
      camIm->imRaw = NULL;
      camIm->imRawType = COLOR_CODING_NONE;
      camIm->im = NULL;
      camIm->imType = COLOR_CODING_NONE;
      camIm->imColor = NULL;
      camIm->imColorType = COLOR_CODING_NONE;
      camIm->imRect = NULL;
      camIm->imRectType = COLOR_CODING_NONE;
      camIm->imRectColor = NULL;
      camIm->imRectColorType = COLOR_CODING_NONE;


      camIm->imWidth = camFrame->size[0];
      camIm->imHeight = camFrame->size[1];
      camIm->im_time = camFrame->timestamp;

      camIm->imRaw = camFrame->image;
      camIm->imRawType = rawType;
      camIm->imRawSize = camFrame->image_bytes;

      //        PRINTF("Time: %llu", camFrame->timestamp);
    }

#if 0
  if (camFrame != NULL &&
      (camFrame->frames_behind > 1  ||
       dc1394_capture_is_frame_corrupt(dcCam, camFrame) == DC1394_TRUE)
      )
    {
      dc1394_capture_enqueue(dcCam, camFrame);
      camFrame = NULL;
    }
#endif

  return (camFrame != NULL);
}



void
dcam::Dcam::setCapturePolicy(dc1394capture_policy_t p)
{
  camPolicy = p;
}


// Features

bool
dcam::Dcam::hasFeature(dc1394feature_t feature)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present),
                   "Could not check if feature was present");
  return (present == DC1394_TRUE);
}

void
dcam::Dcam::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  CHECK_READY();
  if (feature == DC1394_FEATURE_WHITE_BALANCE)
    {
      CHECK_ERR_CLEAN( dc1394_feature_whitebalance_set_value(dcCam, value, value2), "Could not set feature");
    }
  else
    {
      CHECK_ERR_CLEAN( dc1394_feature_set_value(dcCam, feature, value), "Could not set feature");
    }
}

void
dcam::Dcam::getFeatureBoundaries(dc1394feature_t feature, uint32_t& min, uint32_t& max)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present),
                   "Could not check if feature was present");
  if (present == DC1394_TRUE)
    {
      CHECK_ERR_CLEAN( dc1394_feature_get_boundaries(dcCam, feature, &min, &max),
                       "Could not find feature boundaries");
    }
}

void
dcam::Dcam::setFeatureAbsolute(dc1394feature_t feature, float value)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present),
                   "Could not check if feature was present");
  if (present == DC1394_TRUE)
    {
      CHECK_ERR_CLEAN( dc1394_feature_set_absolute_control(dcCam, feature,  DC1394_ON),
                       "Could not enable absolute control.");
      CHECK_ERR_CLEAN( dc1394_feature_set_absolute_value(dcCam, feature, value),
                       "Could not set feature");
    }
}

void
dcam::Dcam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_feature_set_mode(dcCam, feature, mode), "Could not set feature");
}


void
dcam::Dcam::setRegister(uint64_t offset, uint32_t value)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_set_control_register(dcCam, offset, value),
                   "Could not set control register");
}

uint32_t
dcam::Dcam::getRegister(uint64_t offset)
{
  CHECK_READY();
  uint32_t value;
  CHECK_ERR_CLEAN( dc1394_get_control_register(dcCam, offset, &value),
                   "Could not get control register");
  return value;
}


// STOC modes

bool
dcam::Dcam::setProcMode(videre_proc_mode_t mode)
{
  CHECK_READY();
  if (!isSTOC)
    return false;

  procMode = mode;

  // set it while running
  uint32_t qval1 = 0x08000000 | (0x90 << 16) | ( ( mode & 0x7) << 16);
  uint32_t qval2 = 0x08000000 | (0x9C << 16);

  setRegister(0xFF000, qval1);
  setRegister(0xFF000, qval2);

  // set it while stopped
  uint32_t qval = (stocFirmware << 8) | procMode;
  setRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_PROC_OFFSET,qval);

  setRawType();                 // set up image type

  return true;
}


// Raw type
void
dcam::Dcam::setRawType()
{
  if (isSTOC)
    {
      switch (procMode)
        {
        case PROC_MODE_OFF:
        case PROC_MODE_NONE:
        case PROC_MODE_TEST:
          if (isColor)
            rawType = VIDERE_STOC_RAW_RAW_GRBG;
          else
            rawType = VIDERE_STOC_RAW_RAW_MONO;
          break;

        case PROC_MODE_RECTIFIED:
          rawType = VIDERE_STOC_RECT_RECT;
          break;

        case PROC_MODE_DISPARITY:
          rawType = VIDERE_STOC_RECT_DISP;
          break;

        case PROC_MODE_DISPARITY_RAW:
          if (isColor)
            rawType = VIDERE_STOC_RAW_DISP_GRBG;
          else
            rawType = VIDERE_STOC_RECT_DISP; // This is not what it SHOULD be, but what in fact comes out of the camera
          break;
        }
    }
  else if (isVidereStereo) // stereo device
    {
      camIm->imRaw = camFrame->image;
      if (isColor)
        rawType = VIDERE_STEREO_GRBG;
      else
        rawType = VIDERE_STEREO_MONO;
    }
  else
    {
      PRINTF("Setting type of video mode to %d", videoMode);
      switch (videoMode)
        {
        case DC1394_VIDEO_MODE_640x480_RGB8:
          rawType = COLOR_CODING_RGB8;
          break;
        case DC1394_VIDEO_MODE_640x480_MONO8:
          rawType = COLOR_CODING_MONO8;
          break;
        default:
          rawType = COLOR_CODING_MONO8;
        }
    }
}


// Parameters
char *
dcam::Dcam::retParameters()
{
  return camIm->params;
}

bool
dcam::Dcam::putParameters(char *bb)
{
  if (camIm->params)
    delete [] camIm->params;
  int n = strlen(bb);
  char *str = new char[n+1];
  strcpy(str,bb);
  camIm->params = str;
  return true;
}

char *
dcam::Dcam::getParameters()
{
  if (camIm->params)
    free(camIm->params);

  uint32_t qval = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET);
  if (qval == 0xffffffff)
    camIm->params = NULL;
  else
    {
      char *buf = new char[4096*4];
      int n = 4096*4;
      char* bb = buf;

      // read in each byte
      int pos = 0;
      uint32_t quad;
      quad = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos);

      while (quad != 0x0 && quad != 0xffffffff && n > 3)
        {
          int val;
          pos += 4;
          n -= 4;
          val = (quad >> 24) & 0xff;
          *bb++ = val;
          val = (quad >> 16) & 0xff;
          *bb++ = val;
          val = (quad >> 8) & 0xff;
          *bb++ = val;
          val = quad & 0xff;
          *bb++ = val;
          quad = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos);
        }
      *bb = 0;         // just in case we missed the last zero
      camIm->params = buf;
    }
  return camIm->params;
}


// set the calibration and image parameters on a camera

bool
dcam::Dcam::setParameters()
{
  if (!camIm->params)
    return false;

  PRINTF(camIm->params);

  // check firmware version
  if (camFirmware < 0x0201)
    {
      PRINTF("Firmware version absent or too low");
      return false;
    }

  // erase any previous calibration
  uint32_t qval;
  qval = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET);
  //  PRINTF("[Dcam] Calibration start: 0x%08X", qval);
  if (qval == 0xffffffff)
    { PRINTF("No calibration parameters"); }
  else
    {
      PRINTF("[Dcam] Erasing calibration parameters");
      int i;
      for (i=0; i<VIDERE_CALIB_SIZE; i+=512)
        {
          setRegister(VIDERE_LOCAL_BASE,VIDERE_CAM_STORE_MAGIC_NUM);
          setRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+i, ~VIDERE_CAM_STORE_MAGIC_NUM);
          usleep(100000);
        }
    }


  // write out each byte
  int pos = 0;
  uint32_t quad = 0;
  char *bb = camIm->params;
  int n = strlen(bb);
  PRINTF("[Dcam] Writing %d bytes", n);


  while (n--)
    {
      int b = *bb++;
      if (pos > VIDERE_CALIB_SIZE-10)
        {
          PRINTF("[SetCalib] Calibration file too large");
          return false;
        }
      int n = pos % 4;
      int p = (3-n)*8;
      quad |= b<<p;
      if (n==3)
        {
          setRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos-3, quad);
          quad = 0;
        }
      pos++;
    }
  PRINTF("[SetCalib] Wrote %d bytes", pos);
  pos = pos - (pos % 4);
  setRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos, quad);
  setRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos+4, 0x0);
  return true;
}


// upload the parameters and firmware to a STOC device
// erases EEPROM first

// #define VERIFY_EEPROM
bool store_eeprom_bytes(int addr, unsigned char *buf, int count);

bool
dcam::Dcam::setSTOCParams(uint8_t *cbuf, int cn, uint8_t *lbuf, int ln,
                          uint8_t *rbuf, int rn)
{
  if (!isSTOC)
    return false;

  uint32_t qval;
  int v;

  // turn off FPGA
  qval = 0x0D100000;            // switch off FPGA
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  qval = 0x0D120000;            // switch on EEPROM

  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  PRINTF("[Device] Switched off FPGA; switched on EEPROM");

  // erase the flash
  PRINTF("[Device] Erasing flash");
  qval = 0x0D030000; // erase command
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);

  for (int i=0; i<20; i++)
    {
      usleep(1000000);          // wait
      qval = 0x0F000000;
      setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
      usleep(10000);
      qval = getRegister(VIDERE_LOCAL_PARAM_BASE);
      v = qval & 0xff;
      PRINTF("[Device] Read: %02x", v);
      if (v == 0xff) break;
    }

  if (v != 0xff)
    {
      PRINTF("[Device] Couldn't erase flash!");
      return false;
    }

  PRINTF("[Device] Flash erased");

  // write and verify at addr 0
  bool success = store_eeprom_bytes(0, cbuf, cn);
  if (!success)
    {
      PRINTF("[Device] Failed on FPGA configuration");
      goto failconfig;
    }

  // check for warping
  if (ln == 0)                  // no warping, return
    {
      PRINTF("[Device] No warp table, exiting");
      return true;
    }

  // now do left warp table
  PRINTF("[Device] Saving %d bytes to STOC", ln);
  success = store_eeprom_bytes(0x040000, lbuf, ln);
  if (!success)
    {
      PRINTF("[Device] Failed to save warp table to STOC");
      goto failwarp;
    }


  // right warp table
  PRINTF("[Device] Saving %d bytes to STOC", rn);
  success = store_eeprom_bytes(0x060000, rbuf, rn);
  if (!success)
    {
      PRINTF("[Device] Failed to save warp table to STOC");
      goto failwarp;
    }

  // restore FPGA operation
 failconfig:
 failwarp:
  qval = 0x0D130000;            // switch off EEPROM
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  qval = 0x0D110000;            // switch on FPGA
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  PRINTF("[Device] Re-configured FPGA");
  usleep(2000000);
  qval = 0x0D120000;            // switch on EEPROM
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);

  return true;
}


// companding and HDR

bool
dcam::Dcam::setCompanding(bool on)
{
  usleep(50000);

  if (on)
    setRegister(0xFF000, 0x041C0003);
  else
    setRegister(0xFF000, 0x041C0002);

  return true;
}

bool
dcam::Dcam::setHDR(bool on)
{
  usleep(50000);

  if (on)
    setRegister(0xFF000, 0x040F0051);
  else
    setRegister(0xFF000, 0x040F0011);

  return true;
}


//
// value boundaries are given by the max/min variables
//

void
dcam::Dcam::setExposure(int val, bool isauto)
{
  usleep(50000);

  uint32_t v;
  if (val < 0) v = 0;
  else v = val;

  if (v < expMin)
    v = expMin;
  if (v > expMax)
    v = expMax;

  if (isauto)
    setFeatureMode(DC1394_FEATURE_EXPOSURE,DC1394_FEATURE_MODE_AUTO);
  else
    setFeature(DC1394_FEATURE_EXPOSURE,v);      // ??? do we have to set manual here ???
}


void
dcam::Dcam::setGain(int val, bool isauto)
{
  usleep(50000);

  uint32_t v;
  if (val < 0) v = 0;
  else v = val;

  if (v < gainMin)
    v = gainMin;
  if (v > gainMax)
    v = gainMax;

  if (isauto)
    setFeatureMode(DC1394_FEATURE_GAIN,DC1394_FEATURE_MODE_AUTO);
  else
    setFeature(DC1394_FEATURE_GAIN,v);      // ??? do we have to set manual here ???
}

//
//
//
bool
dcam::Dcam::setMaxAutoVals(int exp, int gain)
{
  usleep(50000);

  uint32_t v;
  if (exp < 1) exp = 1;
  if (((uint32_t)exp) > gainMax) exp = gainMax;
  v = 0x04BD0000 | exp;
  setRegister(0xFF000, v);

  if (gain < 0) gain = 0;
  if (((uint32_t)gain) > gainMax) gain = gainMax;
  v = 0x04360000 | (gain+16);
  setRegister(0xFF000, v);

  return true;
}


// brightness

void
dcam::Dcam::setBrightness(int val, bool isauto)
{
  usleep(50000);

  uint32_t v;
  if (val < 0) v = 0;
  else v = val;

  if (v < brightMin)
    v = brightMin;
  if (v > brightMax)
    v = brightMax;

  if (isauto)
    {
      setFeatureMode(DC1394_FEATURE_BRIGHTNESS,DC1394_FEATURE_MODE_AUTO);
    }

  else
    {
#if 0                           // this doesn't seem to have any effect...
      if (v > 0)                // set the brightness target too
        {
          if (v > brightMax-6) v = brightMax-6;
          v = 0xff & v;
          v = 0x04460000 | v | ((v+6)<<8);
          PRINTF("setting v");
          v = 0x0446fff0;
          setRegister(0xFF000, v);
          usleep(50000);
          setRegister(0xFF000, 0x04478080);
        }
#endif
      setFeature(DC1394_FEATURE_BRIGHTNESS,v);   //   ??? do we have to set manual here ???
    }
}



//
// upload bytes to on-camera EEPROM
//

volatile int xx = 0x1a2b3c4d;

bool
dcam::Dcam::store_eeprom_bytes(int addr, uint8_t *buf, int count)
{
  unsigned long qval;
  int addrhigh = addr & 0x00ff0000;
  int addrlow  = (addr & 0x0000ffff) << 8;
  int progress;
  int totprog = count / (200*16);
  unsigned char *cptr = buf;
  int v;

  PRINTF("[Device] Writing %d bytes to address %06x", count, addr);

  setRegister(VIDERE_LOCAL_PARAM_BASE,0x0D120000); // turn on EEPROM

  // set up addr
  qval = 0x09000000 | addrhigh;
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  qval = 0x0A000000 | addrlow;
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);

  // write out bytes
  progress = 0;
  for (int i=count; i>0; i-=16)
    {
      int b0, b1;
      if (i < 16)
        {
          // last bytes
          while (i > 1)
            {
              // get next 2 bytes
              b0 = *cptr++;
              b1 = *cptr++;
              // write them out and save them in buffer for verify
              usleep(1000);
              qval = 0x0B000000 | (b0 << 16) | (b1 << 8);
              setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
              i -= 2;
            }
          break;
        }
      for (int j=0; j<8; j++)
        {
          // get next 2 bytes
          b0 = *cptr++;
          b1 = *cptr++;
          // write them out and save them in buffer for verify
          qval = 0x0C000000 | (b0 << 16) | (b1 << 8);
          setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
        }
      // check that we've completed
      bool success = false;
      for (int j=0; j<5; j++)
        {
          qval = getRegister(VIDERE_LOCAL_PARAM_BASE);
          v = qval & 0xff000000;
          if (v == 0)
            {
              v = qval & 0x00ff0000;
              if (v == 0)
                success = true;
              else
                success = false;
              break;
            }
          usleep(1000);
        }
      if (!success)
        {
          PRINTF("[Device] Failed to complete flash write");
          setRegister(VIDERE_LOCAL_PARAM_BASE,0x0D130000); // turn off EEPROM
          return false;
        }
      progress++;
      if ((progress % 200) == 0)
        {
          PRINTF("[Device] Count %d of %d", progress/200, totprog);
        }
    }
  PRINTF("[Device] Finished storing at address %06x, verifying...", addr+count/2);

  // Now do a verify
  usleep(10000);

#ifdef VERIFY_EEPROM
  // reset the read address
  qval = 0x09000000 | addrhigh;
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  usleep(10000);
  qval = 0x0A000000 | addrlow;
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  usleep(10000);

  // set up read addr in eeprom
  qval = 0x0E000000;
  setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
  usleep(10000);

  // ok, now read each 2 bytes, compare them
  cptr = buf;
  int errcnt = 0;
  for (int i=0; i<count; i+=2)
    {
      // do a read
      setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0F000000);
      // get result
      int pcount = 200;
      //      usleep(3000);
      for (int j=0; j<5; j++)
        {
          usleep(pcount);       // usleep doesn't work well here, need a
                                // good way to chew up a short amount of time
                                // need something like udelay()
          pcount += 500;
          qval = getRegister(VIDERE_LOCAL_PARAM_BASE);
          v = qval & 0xff000000;
          if (v == 0)
            break;
          //      usleep(10000);
        }

      if (v != 0)
        {
          PRINTF("[Device] Time out on verify read: %d %x", xx, v);
          setRegister(VIDERE_LOCAL_PARAM_BASE,0x0D130000); // turn off EEPROM
          return false;
        }
      // done, get result
      v = (qval & 0xff00) >> 8;
      if (v != *cptr)
        {
          PRINTF("[Device] Wrote %02x, read %02x at addr 0x%06x (byte %d)",
                 *cptr, v, i/2+addr, i+addr*2);
          errcnt++;
          //      return;
        }
      cptr++;
      v = qval & 0xff;
      if (v != *cptr)
        {
          PRINTF("[Device] Wrote %02x, read %02x at addr 0x%06x (%d)",
                 *cptr, v, i/2+addr, i+1+addr*2);
          errcnt++;
          //      return;
        }
      cptr++;

      if (errcnt > 20)
        {
          PRINTF("[Device] Errcnt >20, failing verify %d", xx);
          setRegister(VIDERE_LOCAL_PARAM_BASE,0x0D130000); // turn off EEPROM
          return false;
        }

      if (i != 0 && (i%(200*16)) == 0)
        {
          PRINTF("[Device] Verified %d of %d", i/(200*16), totprog);
        }
    }
  PRINTF("[Device] Verified!");
#endif

  setRegister(VIDERE_LOCAL_PARAM_BASE,0x0D130000); // turn off EEPROM
  return true;
}


//
// generate an incremental rectification table,
//   suitable for loading onto a STOC device
//

#define SUBPIX 64.0

int
dcam::Dcam::getIncRectTable(uint8_t *dest)
{
#if 0
  // pixskip is the number of bytes between pixels in the same image.
  // by default, pixels are 8-bit, non-interleaved
  int pixskip = 1;

  int width = sp->linelen;
  int height = sp->lines;

  // output file size
  int outx = width;
  int outy = height;

  // hold pixel increment bits
  int pixinc, pcnt;

  PRINTF("[Warp Table]  Input image size: %d %d", width, height);
  PRINTF("[Warp Table] Output image size: %d %d", outx, outy);

  int count = 0;                // count of bytes
  int i, j;
  float x, y, rnd;
  int ox, oy, odx, ody, dx, dy, hx, hy;
  //  int xmax = 0, ymax = 0, xmin = 0, ymin = 0;
  int px, py;
  double xmax = 0, ymax = 0;
  double ex,ey;

  for (i=0; i<outy; i++)
    {
      dx = dy = 0;
      hx = hy = 0;
      pixinc = 0;
      pcnt = 0;
      for (j=0; j<outx; j++)
        {
          origAddr(&x, &y, (float)j, (float)i, sp, which);
          if (x > 0) rnd = 0.5; else rnd = -0.5;
          px = (int)(x*SUBPIX + rnd);
          if (y > 0) rnd = 0.5; else rnd = -0.5;
          py = (int)(y*SUBPIX + rnd);
          if (j > 0)            // first data point
            {
              dx = px - ox;
              dy = py - oy;
              if (j > 1)
                {
                  // restrict to +-1

                  ex = x - ((double)(ox+odx))/SUBPIX;
                  if (ex > 0)
                    hx = 1;
                  else
                    hx = -1;

                  ey = y - ((double)(oy+ody))/SUBPIX;
                  if (ey > 0)
                    hy = 1;
                  else
                    hy = -1;

                  ex = ex - (double)hx/SUBPIX;
                  if (fabs(ex) > xmax) xmax = fabs(ex);
                  ey = ey - (double)hy/SUBPIX;
                  if (fabs(ey) > ymax) ymax = fabs(ey);

                  dx = odx + hx;
                  dy = ody + hy;

                  // accumulate pixel increment shift
                  pixinc = (pixinc << 2);
                  if (hx == 1)
                    pixinc = pixinc | 0x02;
                  if (hy == 1)
                    pixinc = pixinc | 0x01;
                  // check for writing out
                  pcnt++;
                  if (pcnt >= 4)
                    {
                      *dest++ = pixinc;
                      count++;
                      pcnt = 0;
                      pixinc = 0;
                    }
                }               // j > 1
              else
                {               // j = 1
                  *dest++ = dx;
                  *dest++ = dy;
                  count += 2;
                }
              odx = dx;
              ody = dy;
              ox = ox + dx;
              oy = oy + dy;
            }                   // j > 0
          else
            {                   // j = 0, start of line
              ox = px;
              oy = py;
              *dest++ = (px & 0xff00) >> 8;
              *dest++ = px & 0xff;
              *dest++ = (py & 0xff00) >> 8;
              *dest++ = py & 0xff;
              //              PRINTF("%03d %04x %04x", i, px, py);
              count += 4;
            }
          if (hx > 1 || hx < -1 || hy > 1 || hy < -1)
            PRINTF("[Warp Table] Increment too large");

        } // end of loop over line pixels

      // check if the last pixinc gets written out
      if (pcnt > 0)
        {
          while (pcnt++ < 4)
            pixinc = pixinc << 2;
          *dest++ = pixinc;
          count++;
        }
    }

  PRINTF("[Warp Table] Max X change: %f", xmax);
  PRINTF("[Warp Table] Max Y change: %f", ymax);
  PRINTF("[Warp Table] Size is %d bytes", count);
  return count;
#endif
  return 0;
}
