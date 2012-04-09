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

#ifndef DCAM_H
#define DCAM_H

#include <ros/node_handle.h>
#include <sensor_msgs/image_encodings.h>
#include <stdexcept>
#include <cstring>
#include <dc1394/dc1394.h>
#include "dcam1394/image_proc.h"

// Pixel raw modes
// Videre stereo:
//   Mono has left/right pixels interlaced
//   Color has left/right pixels interlace, bayer pixels
//   STOC modes have rectified images, raw encodings, disparity, etc
//

#ifndef COLOR_CODING_T
typedef enum {
  COLOR_CODING_MONO8 = 3000,
  COLOR_CODING_MONO16,
  COLOR_CODING_BAYER8_RGGB,
  COLOR_CODING_BAYER8_BGGR,
  COLOR_CODING_BAYER8_GBRG,
  COLOR_CODING_BAYER8_GRBG,
  COLOR_CODING_BAYER16_RGGB,
  COLOR_CODING_BAYER16_BGGR,
  COLOR_CODING_BAYER16_GBRG,
  COLOR_CODING_BAYER16_GRBG,
  COLOR_CODING_RGB8,            // RGB order
  COLOR_CODING_RGBA8,           // RGBA order
  COLOR_CODING_RGB16,           // RGB order
  COLOR_CODING_RGBA16,          // RGBA order

  // these are stereo interlace encodings
  // Videre stereo:
  //   Mono has left/right pixels interlaced
  //   Color has left/right pixels interlace, bayer pixels
  //   STOC modes have rectified images, raw encodings, disparity, etc
  VIDERE_STEREO_MONO,
  VIDERE_STEREO_RGGB,
  VIDERE_STEREO_GRBG,
  VIDERE_STEREO_BGGR,
  VIDERE_STOC_RECT_RECT,        // left and right rectified mono
  VIDERE_STOC_RECT_DISP,        // left rectified mono, right disparity
  VIDERE_STOC_RAW_DISP_MONO,    // left raw mono, right disparity
  VIDERE_STOC_RAW_DISP_RGGB,    // left raw color, right disparity
  VIDERE_STOC_RAW_DISP_GRBG,    // left raw color, right disparity
  VIDERE_STOC_RAW_RAW_MONO,     // left and right raw, mono
  VIDERE_STOC_RAW_RAW_RGGB,     // left and right raw, color
  VIDERE_STOC_RAW_RAW_GRBG,     // left and right raw, color


  COLOR_CODING_NONE             // no image info
} color_coding_t;
#define COLOR_CODING_T
#endif


// Frame size
typedef enum {
  SIZE_640x480 = 0,
  SIZE_320x240,
  SIZE_1280x960,
  SIZE_512x384,
  SIZE_1024x768
} size_coding_t;



// STOC modes
typedef enum {
  PROC_MODE_OFF = 0,
  PROC_MODE_NONE,
  PROC_MODE_TEST,               // test image
  PROC_MODE_RECTIFIED,
  PROC_MODE_DISPARITY,
  PROC_MODE_DISPARITY_RAW
} videre_proc_mode_t;


// Videre offsets
#define VIDERE_LOCAL_BASE                     0xF0000UL
#define VIDERE_PARAM1_OFFSET                  0x0400
#define VIDERE_CALIB_OFFSET                   0x0800
#define VIDERE_CALIB_SIZE                     0x1000           //  4 KB
#define VIDERE_USER_OFFSET                    0x1800
#define VIDERE_USER_SIZE                      0x1000           //  4 KB
#define VIDERE_DOWNLOAD_OFFSET                0x4000
#define VIDERE_DOWNLOAD_SIZE                  0x3C00           // 15 KB
#define VIDERE_BLOCK_SIZE                     0x200            // 512 B block size
#define VIDERE_UPGRADE_CONFIG                 0x7A00           // rom config values to be saved
#define VIDERE_UPGRADE_PARAMS                 0x7B00           // local param storage
#define VIDERE_UPGRADE_OFFSET                 0x7BF8           // where magic num is written
#define VIDERE_SCRATCHPAD_OFFSET              0x7C00
#define VIDERE_SCRATCHPAD_SIZE                0x80             // 128 B
#define VIDERE_LOCAL_PARAM_BASE               0xFF000UL
#define VIDERE_CAM_STORE_MAGIC_NUM            0x5A01F687UL
#define VIDERE_CAM_FW_LEVEL_OFFSET            (1*4)
#define VIDERE_CAM_LEVEL_OFFSET               (2*4)
#define VIDERE_CAM_ID_OFFSET                  (3*4)
#define VIDERE_CAM_BLACK_OFFSET               (4*4)
#define VIDERE_CAM_START_ROW_OFFSET           (5*4)
#define VIDERE_CAM_START_COL_OFFSET           (6*4)
#define VIDERE_CAM_IMAGER_SIZE_OFFSET         (7*4)
#define VIDERE_CAM_FRAME_DIV_OFFSET           (8*4)
#define VIDERE_CAM_50HZ_OFFSET                (9*4)
#define VIDERE_CAM_PROC_OFFSET                (10*4)
#define VIDERE_CAM_PROC_THRESH_OFFSET         (11*4)

// define some Videre modes
#define VIDERE_STEREO_1280x960 DC1394_VIDEO_MODE_1280x960_YUV422
#define VIDERE_STEREO_1024x768 DC1394_VIDEO_MODE_1024x768_YUV422
#define VIDERE_STEREO_640x480 DC1394_VIDEO_MODE_640x480_YUV422
#define VIDERE_STEREO_512x384 DC1394_VIDEO_MODE_512x384_YUV422
#define VIDERE_STEREO_320x240 DC1394_VIDEO_MODE_320x240_YUV422


using namespace cam;

namespace dcam
{

  class DcamException : public std::runtime_error
    {
    public:
    DcamException(const char* msg) : std::runtime_error(msg) {}
    };

  void init();                  // initializes the bus
  void fini();                  // releases the bus
  size_t numCameras();          // number of cameras found
  uint64_t getGuid(size_t i);   // camera ids
  char *getVendor(size_t i);
  char *getModel(size_t i);
  extern dc1394_t *dcRef;       // IEEE 1394 system object
  const char *getModeString(dc1394video_mode_t mode); // Mode string from mode

  class Dcam
  {
    friend void init();
    friend void fini();
    friend size_t numCams();
    friend uint64_t getGuid(size_t i);
    friend char *getVendor(size_t i);
    friend char *getModel(size_t i);

  public:
    Dcam(uint64_t guid, size_t bufferSize = 8);

    virtual ~Dcam();

    char *getVendor();          // just our own
    char *getModel();
    virtual dc1394video_modes_t *getModes();

    virtual void setFormat(dc1394video_mode_t video = DC1394_VIDEO_MODE_640x480_MONO8,
                           dc1394framerate_t fps = DC1394_FRAMERATE_30,
                           dc1394speed_t speed = DC1394_ISO_SPEED_400);

    virtual void start();
    virtual void stop();

    ImageData *camIm;           // image data

    virtual bool getImage(int ms); // gets the next image, with timeout
    virtual void setCapturePolicy(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    // general DC1394 interface
    virtual bool hasFeature(dc1394feature_t feature);
    virtual void getFeatureBoundaries(dc1394feature_t feature, uint32_t& min, uint32_t& max);
    virtual void setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2 = 0);
    virtual void setFeatureAbsolute(dc1394feature_t feature, float value);
    virtual void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode);

    // particular features of importance
    virtual void setExposure(int val, bool isauto);
    virtual void setGain(int val, bool isauto);
    virtual void setBrightness(int val, bool isauto);

    // Format7 functions
    uint32_t hsize, vsize, packet_size;
    virtual void setSquareROI(dc1394video_mode_t video);

    // feature boundaries
    uint32_t expMax, expMin;
    uint32_t gainMax, gainMin;
    uint32_t brightMax, brightMin;
    bool setMaxAutoVals(int exp, int gain); // set max for auto gain and exposure algorithm

    // low-level register access
    // implicitly assumes CCR base, so that DCAM register are at offsets
    //   e.g., for EXPOSURE use 0x804
    virtual void setRegister(uint64_t offset, uint32_t value);
    virtual uint32_t getRegister(uint64_t offset);

    virtual bool setProcMode(videre_proc_mode_t mode);

    virtual bool setCompanding(bool on); // bring up low light levels
    virtual bool setHDR(bool on); // high dynamic range

    virtual char *getParameters(); // download from device
    virtual char *retParameters(); // just return current param string
    virtual bool putParameters(char *p); // just set current param string, handle buffering
    virtual bool setParameters(); // upload to device
    virtual bool setSTOCParams(uint8_t *cbuf, int cn, // upload to STOC device
                               uint8_t *lbuf, int ln, // STOC firmware, left and right warp tables
                               uint8_t *rbuf, int rn);

    virtual int  getIncRectTable(uint8_t *buf); // make a warp table for a STOC device
    uint64_t camGUID;           // our own GUID
    uint32_t imFirmware, camFirmware, stocFirmware;
    bool isSTOC, isVidereStereo, isVidere, isColor; // true if a STOC, Videre stereo, color device

  protected:
    bool started;
    size_t bufferSize;          // number of DMA buffers
    dc1394video_modes_t camModes; // valid modes
    dc1394capture_policy_t camPolicy; // current capture policy
    dc1394video_frame_t *camFrame;      // current captured frame
    dc1394camera_t *dcCam;      // the camera object
    virtual void cleanup();
    virtual void setRawType();
    videre_proc_mode_t procMode; // STOC mode, if applicable
    dc1394color_filter_t bayerMode; // bayer color encoding
    dc1394video_mode_t videoMode;
    color_coding_t rawType;     // what type of raw image we receive
    virtual bool store_eeprom_bytes(int addr, uint8_t *buf, int count);
  };

};


#endif
