/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#ifndef UV4L2_H
#define UV4L2_H


// #include <stdio.h>
// #include <stdlib.h>
// 
// 
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <assert.h>
// 
// #include <getopt.h>             /* getopt_long() */
// 
// #include <fcntl.h>              /* low-level i/o */
// #include <unistd.h>
// #include <errno.h>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include <sys/time.h>
// #include <sys/mman.h>
// #include <sys/ioctl.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "utime.h"
// 
// #ifndef V4L2_PIX_FMT_H264
// #define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
// #endif


class UV4l2
{
public:
  bool cameraOpen = false;
protected:
  enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
  };
  const char     * dev_name;
  enum io_method   io = IO_METHOD_MMAP;
  int              force_format;
  int              frame_count = 200; // for test only
  int              frame_number = 0;
  
  int w = 0,h = 0;
  int step = 0;
  unsigned int pixelFormat = 0;
  // resulting image
//   cv::Mat imRGB;
  UTime tImg;

private:

  struct buffer {
    void   *start;
    size_t  length;
  };

  int              fd = -1;
  struct buffer   *buffers;
  unsigned int     n_buffers;
  int              out_buf = 0;

  struct v4l2_queryctrl queryctrl;
  struct v4l2_querymenu querymenu;
  
protected:

  /** convert to cv::Mat image */
  virtual void process_image(void *p, int size, cv::Mat & imRGB);
  /**
  * Close file handle */
  const void close_device(void);
  /**
  * Check if device exist and open a file handle */
  const void open_device(void);
  /**
  * Init device with size and format */
  const void init_device(void);
  /**
  * Init device read method
  * Use memory mapped buffers in user space. */
  const void init_mmap(void);
  /**
  * Stop capturing - tell camera to stop */
  const void stop_capturing(void);
  /**
  * Start capturing - tell camera to start streaming in desired format */
  const void start_capturing(void);
  /**
  * device setting cleanup */
  const void uninit_device(void);
  /**
   * Capture and convert image to RGB.
   * \returns the image or generate an error */
  bool grab(cv::Mat & image);
  /**
   * list video capability */
  void listCapability();
  
  

private:

  const void errno_exit(const char *s);

  /**
  * Sending queries and device settings to device */
  const int xioctl(int fh, unsigned int request, void *arg);

  const int read_frame(cv::Mat & image);
  /**
  * main loop is for API test only */
  const void mainloop(void);


  /**
  * device read method (copy data)
  * - not used */
  const void init_read(unsigned int buffer_size); 
  /**
  * device read using user pointer method
  * - not used */
  const void init_userp(unsigned int buffer_size); // 

  void enumerate_menu (void);

public:
  /**
   * Not tested, use v4l2-ctl -d/dev/video0 --all
   * */
  void setBrightness(int value);
  /**
   * Not tested, use v4l2-ctl -d/dev/video0 --all
   * */
  void setContrast(int value);
  /**
   * */
  void setWhiteBalance(bool toAuto);
  /**
   * set video gain to auto (=0) or to manual gain (gain > 0)
   * see use v4l2-ctl -d/dev/video0 --all
   * for capabilities
   */
  void setGain(int gain);
  /**
   * set exposure 11-183 (big Raspberry camera)
   * */
  void setExposure(int value);
  
  
  /**
  * Old command line API test help (not used) */
  const void usage(FILE *fp, int argc, char **argv);

  // const char short_options[] = "d:hmruofc:";

  // const struct option
  // long_options[] = {
  //   { "device", required_argument, NULL, 'd' },
  //   { "help",   no_argument,       NULL, 'h' },
  //   { "mmap",   no_argument,       NULL, 'm' },
  //   { "read",   no_argument,       NULL, 'r' },
  //   { "userp",  no_argument,       NULL, 'u' },
  //   { "output", no_argument,       NULL, 'o' },
  //   { "format", no_argument,       NULL, 'f' },
  //   { "count",  required_argument, NULL, 'c' },
  //   { 0, 0, 0, 0 }
  // };
public:
  /**
  *API test - not used */  
  int test_library(void);

};

#endif
