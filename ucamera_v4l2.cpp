
#include <stdio.h>
#include <stdlib.h>

/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>

#include <linux/videodev2.h>

#include "ucamera_v4l2.h"
#include "utime.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

const void UV4l2::errno_exit(const char *s)
{
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

const int UV4l2::xioctl(int fh, unsigned int request, void *arg)
{
  int r;
  
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  
  return r;
}

void UV4l2::process_image(void *p, int size, cv::Mat & imRGB)
{
  frame_number++;
  if (false)
  {
    char filename[15];
    sprintf(filename, "frame-%d.raw", frame_number);
    FILE *fp=fopen(filename,"wb");
    
    if (out_buf)
    {
      fwrite(p, size, 1, fp);
      printf("# saved image %d\n", frame_number);
    }
    else
      printf("# processed image %d of %dx%d buf size = %d\n", frame_number, h,w, size);
    fflush(fp);
    fclose(fp);
  }
  cv::Size sz(w,h);
  // destination image
  //imRGB = cv::Mat::zeros(cv::Size(1, 49), CV_64FC1);
  // convert to RGB
  if (pixelFormat == V4L2_PIX_FMT_YUYV)
  {
    cv::Mat imYUY2(sz,CV_8UC2, (void*)p);  
    cv::cvtColor(imYUY2, imRGB, cv::COLOR_YUV2BGR_YUYV);
  }
  else if (pixelFormat == V4L2_PIX_FMT_RGB24)
  {
    cv::Mat im(sz,CV_8UC3, (void*)p);
    imRGB = im.clone();
    printf("# Image %d\n", frame_number);
  }
  else if (pixelFormat == V4L2_PIX_FMT_SBGGR10)
  { // 10 bit Bayer unpacked (BG10)
    // printf("# unpacking V4L2_PIX_FMT_SBGGR10 (10 bit Bayer 'BG10')\n");
    uint8_t * ba8a = (uint8_t *)malloc(size/2);
    uint16_t * ba10a = (uint16_t *)p;
    uint8_t * ba8 = ba8a;
    uint16_t * b1 = ba10a;
    struct timespec tns1;
    struct timespec tns2;
    clock_gettime(CLOCK_REALTIME, & tns1);
    for (int i = 0; i < size/(2*4); i++)
    { // unfolding by 4 pixels in a row (saves a bit of time)
      *ba8++ = *b1++ >> 2; // remove 2 LSB
      *ba8++ = *b1++ >> 2;
      *ba8++ = *b1++ >> 2;
      *ba8++ = *b1++ >> 2;
    }
    clock_gettime(CLOCK_REALTIME, & tns2);
    if (false)
    { // print conversion time
      // calculate in us
      double dt = (tns2.tv_sec - tns1.tv_sec) * 1e6 +  (tns2.tv_nsec - tns1.tv_nsec)*1e-3;
      printf("# 10bit to 8 bit took  %g us\n", dt);
    }
    //
    clock_gettime(CLOCK_REALTIME, & tns1);
    cv::Mat im(sz,CV_8UC1, (void*)ba8a);  
    cv::demosaicing(im, imRGB, cv::COLOR_BayerRG2BGR, 3);
    clock_gettime(CLOCK_REALTIME, & tns2);
    if (false)
    { // print conversion time
      // calculate in us
      double dt = (tns2.tv_sec - tns1.tv_sec) * 1e6 +  (tns2.tv_nsec - tns1.tv_nsec)*1e-3;
      printf("# DeBayer took          %g us\n", dt);
    }
    free(ba8a);
  }
  else if(pixelFormat == V4L2_PIX_FMT_SBGGR8)
  { // 8-bit Bayer (BA81)
    printf("# unpacking V4L2_PIX_FMT_SBGGR8 (8 bit Bayer BA81) - not working with new camera\n");
    cv::Mat im(sz,CV_8UC1, (void*)p);  
    cv::demosaicing(im, imRGB, cv::COLOR_BayerRG2BGR, 3);
  }
  else if (pixelFormat == V4L2_PIX_FMT_SBGGR10P)
  { // 10 bit packed Bayer (pBAA)
    printf("# unpacking V4L2_PIX_FMT_SBGGR10P (10 bit packed Bayer 'pBAA') -- not working with new camera\n");
    uint8_t * ba8a = (uint8_t *)malloc(size/2);
    uint16_t * ba10a = (uint16_t *)p;
    uint8_t * ba8 = ba8a;
    uint16_t * b1 = ba10a;
    uint16_t * b2 = ba10a + 1;
    const int n = w*h;
    int b = 0;
    unsigned int v = 0;
    for (int i = 0; i < n; i++)
    { // convert from 10 bit to 8 bit
      switch (b)
      {
        case 0:
          v = *b1 >> 6;
          *ba8 = v >> 2;
          break;
        case 1:
          v = (*b1 << 4) + (*b2 >> 12);
          *ba8 = v >> 2;
          break;
        case 2:
          b1++;
          b2++;
          v = *b1 >> 2;
          *ba8 = v >> 2;
          break;
        case 3:
          v = (*b1 << 8) + (*b2 >> 8);
          *ba8 = v >> 2;
          break;
        case 4:
          b1++;
          b2++;
          v = (*b1 << 2) + (*b2 >> 14);
          *ba8 = v >> 2;
          break;
        case 5:
          v = (*b2 >> 4);
          *ba8 = v >> 2;
          break;
        case 6:
          b1++;
          b2++;
          v = (*b1 << 6) + (*b2 >> 10);
          *ba8 = v >> 2;
          break;
        case 7:
          b1++;
          b2++;
          v = *b1;
          *ba8 = v >> 2;
          break;
      }
      if (i < 50)
      {
        printf("# pixel %d, set %d, b1=%x, b2=%x is %x -> %x\n", i, b, *b1, *b2, v, *ba8);
      }
      ba8++;
      b++;
      if (b > 7)
      {
        b = 0;
        b1++;
        b2++;
      }
    }
    // now we have 1 byte per pixel Bayer-coded 
    //  0   BGBGBGBG ...
    //  1   GRGRGRGR ...
//     cv::Size pxs(w,h);
    cv::Mat src(sz, CV_8UC1, (void*)ba8a);
    cv::demosaicing(src, imRGB, cv::COLOR_BayerRG2BGR,3);
    free(ba8a);
  }
  if (false)
  { // if robot has screen (or connected with -X)
    cv::imshow("Display window", imRGB);
    if (frame_number == 100)
    { // wait for key?
      printf("Press 's' to save image (size=%dx%d)\n", imRGB.cols, imRGB.rows);
      int k = cv::waitKey(0);
      bool isOK = false;
      if (k == 's')
      {
        isOK = cv::imwrite("mmm.png", imRGB);
        printf("Saved image (ok=%d)\n", isOK);
      }
      else
        printf("Not saved image (k=%c)\n", k);
    }
  }
  else
  {
//     printf("# mage converted to BGR - not saved\n");
  }
}

const int UV4l2::read_frame(cv::Mat & image)
{
  struct v4l2_buffer buf;
  unsigned int i;
  
  switch (io) {
    case IO_METHOD_READ:
      if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
        switch (errno) {
          case EAGAIN:
            return 0;
            
          case EIO:
            /* Could ignore EIO, see spec. */
            
            /* fall through */
            
            default:
              errno_exit("read");
        }
      }
      
      process_image(buffers[0].start, buffers[0].length, image);
      break;
      
    case IO_METHOD_MMAP:
      CLEAR(buf);
      
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      
      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;
            
          case EIO:
            /* Could ignore EIO, see spec. */
            
            /* fall through */
            
            default:
              errno_exit("VIDIOC_DQBUF");
        }
      }
      
      assert(buf.index < n_buffers);
      
      process_image(buffers[buf.index].start, buf.bytesused, image);
      
      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
      break;
      
    case IO_METHOD_USERPTR:
      CLEAR(buf);
      
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      
      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;
            
          case EIO:
            /* Could ignore EIO, see spec. */
            
            /* fall through */
            
            default:
              errno_exit("VIDIOC_DQBUF");
        }
      }
      
      for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long)buffers[i].start
          && buf.length == buffers[i].length)
          break;
        
        assert(i < n_buffers);
      
      process_image((void *)buf.m.userptr, buf.bytesused, image);
      
      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
      break;
  }
  
  return 1;
}


bool UV4l2::grab(cv::Mat & image)
{ // wait for data is ready (up to 2 seconds)
  // then read frame and convert to RGB 
  bool isOK = false;
  for (;;) {
    fd_set fds;
    struct timeval tv;
    int r;
    // set device to check
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    // wait for data available
    r = select(fd + 1, &fds, NULL, NULL, &tv);
    // test for signal - retry to get image
    if (-1 == r) {
      if (EINTR == errno)
        continue;
      errno_exit("select");
    }
    // is camera still there (other error)
    if (0 == r) {
      fprintf(stderr, "select timeout\n");
      exit(EXIT_FAILURE);
    }
    // read and process data
    tImg.now();
    isOK = read_frame(image);
    break;
    /* EAGAIN - continue select loop. */
  }
  return isOK;
}

const void UV4l2::mainloop(void)
{ // not used (library test only)
  unsigned int count;
  
  count = frame_count;
  cv::Mat imRGB;
  
  while (count-- > 0) {
    for (;;) {
      fd_set fds;
      struct timeval tv;
      int r;
      
      FD_ZERO(&fds);
      FD_SET(fd, &fds);
      
      /* Timeout. */
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      
      r = select(fd + 1, &fds, NULL, NULL, &tv);
      
      if (-1 == r) {
        if (EINTR == errno)
          continue;
        errno_exit("select");
      }
      
      if (0 == r) {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
      }
      
      if (read_frame(imRGB))
      {
        break;
      }
      /* EAGAIN - continue select loop. */
    }
  }
}

const void UV4l2::stop_capturing(void)
{
  enum v4l2_buf_type type;
  
  switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;
      
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
      break;
  }
}

const void UV4l2::start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;
  
  switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;
      
    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;
        
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON 1");
      break;
      
    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;
        
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers[i].start;
        buf.length = buffers[i].length;
        
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON 2");
      break;
  }
  // 
  // set camera control
  if (false)
  {
    v4l2_control camCtrl;
    camCtrl.id = V4L2_CID_EXPOSURE_AUTO;
    camCtrl.value = V4L2_EXPOSURE_AUTO;
    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &camCtrl))
    {
      printf("# failed to set auto exposure ------------------- bad\n");
    }
  }  
}

const void UV4l2::uninit_device(void)
{
  unsigned int i;
  
  switch (io) {
    case IO_METHOD_READ:
      free(buffers[0].start);
      break;
      
    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
      {
        if (-1 == munmap(buffers[i].start, buffers[i].length))
          errno_exit("munmap");
      }
      break;
      
    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i)
        free(buffers[i].start);
      break;
  }
  
  free(buffers);
}

const void UV4l2::init_read(unsigned int buffer_size)
{
  buffers = (struct buffer *)calloc(1, sizeof(*buffers));
  
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  
  buffers[0].length = buffer_size;
  buffers[0].start = malloc(buffer_size);
  
  if (!buffers[0].start) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
}

const void UV4l2::init_mmap(void)
{
  struct v4l2_requestbuffers req;
  
  CLEAR(req);
  
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support "
      "memory mapping\n", dev_name);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  
  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n",
            dev_name);
    exit(EXIT_FAILURE);
  }
  
  buffers = (struct buffer *)calloc(req.count, sizeof(*buffers));
  
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  
  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;
    
    CLEAR(buf);
    
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;
    
    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");
    
    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
    mmap(NULL /* start anywhere */,
         buf.length,
         PROT_READ | PROT_WRITE /* required */,
         MAP_SHARED /* recommended */,
         fd, buf.m.offset);
    
    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

const void UV4l2::init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  
  CLEAR(req);
  
  req.count  = 4;
  req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support "
      "user pointer i/o\n", dev_name);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  
  buffers = (struct buffer *)calloc(4, sizeof(*buffers));
  
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  
  for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = malloc(buffer_size);
    
    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }
  }
}

const void UV4l2::init_device(void)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  
  
  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n",
              dev_name);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n",
            dev_name);
    exit(EXIT_FAILURE);
  }
  
  switch (io) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        fprintf(stderr, "%s does not support read i/o\n",
                dev_name);
        exit(EXIT_FAILURE);
      }
      break;
      
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s does not support streaming i/o\n",
                dev_name);
        exit(EXIT_FAILURE);
      }
      printf("# %s Camera support streaming\n", dev_name);
      break;
  }
  
  
  /* Select video input, video standard and tune here. */
  
  
  CLEAR(cropcap);
  
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */
    
    if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
    printf("# no CROP capability - ignored\n");
  }
  
  
  CLEAR(fmt);
  
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (force_format) 
  {
//     fmt.fmt.pix.width       = 640; //replace
//     fmt.fmt.pix.height      = 480; //replace
    fmt.fmt.pix.width       = w; //replace
    fmt.fmt.pix.height      = h; //replace
    fmt.fmt.pix.pixelformat = pixelFormat; //replace
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;
    //
    char * p1 = (char*) &fmt.fmt.pix.pixelformat;
    fprintf(stderr, "Setting format 0x%x and size (%d,%d) %c%c%c%c\r\n", fmt.fmt.pix.pixelformat, w, h, p1[0], p1[1], p1[2], p1[3]);
    //
    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
    xioctl(fd, VIDIOC_G_FMT, &fmt);
    p1 = (char*) &fmt.fmt.pix.pixelformat;
    fprintf(stderr, "Getting format 0x%x and size (%d,%d) %c%c%c%c\r\n", fmt.fmt.pix.pixelformat, w, h, p1[0], p1[1], p1[2], p1[3]);
    /* Note VIDIOC_S_FMT may change width and height. */
    if (fmt.fmt.pix.pixelformat != pixelFormat)
    {
      printf("# ---- Failed to get anticipated pixel-format - will not work\n");
      close_device();
    }
  } 
  else 
  {
    /* Preserve original settings as set by v4l2-ctl for example */
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
    else
    {
      unsigned int f = fmt.fmt.pix.pixelformat;
      w = fmt.fmt.pix.width;
      h = fmt.fmt.pix.height;
      step = fmt.fmt.meta.buffersize;
      printf("# format w,h=%d,%d) fmt=%d, buffer %d\n", w,h,f, step);
    }
  }
  
  if (cameraOpen)
  {
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
      fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
      fmt.fmt.pix.sizeimage = min;


    
    switch (io) {
      case IO_METHOD_READ:
        init_read(fmt.fmt.pix.sizeimage);
        break;
        
      case IO_METHOD_MMAP:
        init_mmap();
        break;
        
      case IO_METHOD_USERPTR:
        init_userp(fmt.fmt.pix.sizeimage);
        break;
    }
    //   listCapability();
    //   setWhiteBalance(true);
    //   setGain(0 /* 0=auto*/);
    //   setExposure(1010); // 4--1183 (step 1)
    //   // setBrightness(1000);
    //   // setContrast(1000);
  }  
}

const void UV4l2::close_device(void)
{
  //if (-1 == close(fd))
  //   errno_exit("close");
  if (cameraOpen)
    close(fd);
  fd = -1;
    cameraOpen = false;
}

const void UV4l2::open_device(void)
{
  struct stat st;
  
  if (-1 == stat(dev_name, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n",
            dev_name, errno, strerror(errno));
//     exit(EXIT_FAILURE);
  }
  else
  {
  
    if (!S_ISCHR(st.st_mode)) {
      fprintf(stderr, "%s is no device\n", dev_name);
      //exit(EXIT_FAILURE);
    }
    else
    {
      fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
      
      if (-1 == fd) {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
    //     exit(EXIT_FAILURE);
      }
      else
                cameraOpen = true;
    }
  }
}

void UV4l2::enumerate_menu (void)
{
  printf ("  Menu items:\n");
  //
  memset (&querymenu, 0, sizeof (querymenu));
  querymenu.id = queryctrl.id;
  //
  for (querymenu.index = queryctrl.minimum; int(querymenu.index) <= queryctrl.maximum;  querymenu.index++) 
  {
    if (0 == ioctl (fd, VIDIOC_QUERYMENU, &querymenu)) 
    {
      printf ("  %s\n", querymenu.name);
    } 
    else 
    {
      perror ("VIDIOC_QUERYMENU");
      exit (EXIT_FAILURE);
    }
  }
}


void UV4l2::listCapability()
{
  if (false)
  { // not working
    v4l2_std_id std_id;
    struct v4l2_standard standard;
    //
    if (-1 == ioctl(fd, VIDIOC_G_STD, &std_id)) {
      /* Note when VIDIOC_ENUMSTD always returns ENOTTY this
      *       is no video device or it falls under the USB exception,
      *       and VIDIOC_G_STD returning ENOTTY is no error. */
      
      perror("VIDIOC_G_STD");
      exit(EXIT_FAILURE);
    }
    // 
    memset(&standard, 0, sizeof(standard));
    standard.index = 0;
    
    while (0 == ioctl(fd, VIDIOC_ENUMSTD, &standard)) {
      if (standard.id & std_id) {
        printf("Current video standard: %s\\n", standard.name);
        exit(EXIT_SUCCESS);
      }
      
      standard.index++;
    }
    
    /* EINVAL indicates the end of the enumeration, which cannot be
    *   empty unless this device falls under the USB exception. */
    
    if (errno == EINVAL || standard.index == 0) {
      perror("VIDIOC_ENUMSTD");
      exit(EXIT_FAILURE);
    }
  }
  if (false)
  { // not working
    v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    printf("# is OK?\n");
    if (-1 == ioctl(fd, VIDIOC_ENUM_FMT, &cap)) {
      /* Note when VIDIOC_ENUMSTD always returns ENOTTY this
       *       is no video device or it falls under the USB exception,
       *       and VIDIOC_G_STD returning ENOTTY is no error. */
      
      perror("VIDIOC_ENUM_FMT");
      exit(EXIT_FAILURE);
    }
    printf("# is OK\n");
  }
  if (true)
  {
    memset (&queryctrl, 0, sizeof (queryctrl));
    
    for (queryctrl.id = V4L2_CID_BASE; queryctrl.id < V4L2_CID_LASTP1; queryctrl.id++) 
    {
      if (0 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
        if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
          continue;
        
        printf ("Control CID  %s\n", queryctrl.name);
        
        if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
          enumerate_menu ();
      } else {
        if (errno == EINVAL)
          continue;
        
        perror ("VIDIOC_QUERYCTRL");
        exit (EXIT_FAILURE);
      }
    }
    
    for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;queryctrl.id++) 
    {
      if (0 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) 
      {
        if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
          continue;
        
        printf ("Control PRIV %d %s\n", queryctrl.id, queryctrl.name);
        
        if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
          enumerate_menu ();
      } 
      else 
      {
        if (errno == EINVAL)
          break;
        
        perror ("VIDIOC_QUERYCTRL");
        exit (EXIT_FAILURE);
      }
    }
  }
}


void UV4l2::setBrightness(int value)
{
  /*
   * Driver Info: (big raspberry-pi camera)
   *        Driver name      : unicam
   *        Card type        : unicam
   *        Bus info         : platform:fe801000.csi
   *        Driver version   : 5.10.63
   *        Capabilities     : 0x85a00001
   * User Controls
   * 
   *        white_balance_automatic 0x0098090c (bool)   : default=0 value=1
   *                       exposure 0x00980911 (int)    : min=4 max=1100 step=1 default=1000 value=1000
   *                 gain_automatic 0x00980912 (bool)   : default=0 value=1
   * 
   * Camera Controls
   * 
   *                  auto_exposure 0x009a0901 (menu)   : min=0 max=1 default=1 value=1
   *                                0: Auto Mode
   *                                1: Manual Mode
   * */
  // see http://v4l-test.sourceforge.net/spec/x542.htm for list of options
  // use 
  // $ v4l2-ctl -d/dev/video0 --all
  // for list of capabilities and current values
//   struct v4l2_queryctrl queryctrl;
  struct v4l2_control control;
  //
  if (true)
  { // brightness
    memset (&queryctrl, 0, sizeof (queryctrl));
    queryctrl.id = V4L2_CID_BRIGHTNESS;

    if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) 
    {
      if (errno != EINVAL) 
      {
        perror ("VIDIOC_QUERYCTRL");
        exit (EXIT_FAILURE);
      } 
      else 
      {
        printf ("V4L2_CID_BRIGHTNESS is not supported\n");
      }
    } 
    else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) 
    {
      printf ("V4L2_CID_BRIGHTNESS is not supported\n");
    } 
    else 
    {
      printf("Setting brightness to (default=%d)\n", queryctrl.default_value);
      memset (&control, 0, sizeof (control));
      control.id = V4L2_CID_BRIGHTNESS;
      control.value = queryctrl.default_value;
      
      if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
        perror ("VIDIOC_S_CTRL");
        exit (EXIT_FAILURE);
      }
    }
  }
}

////////////////////////////////////////////////////

void UV4l2::setContrast(int value)
{
  //struct v4l2_queryctrl queryctrl;
  struct v4l2_control control;
  memset (&control, 0, sizeof (control));
  control.id = V4L2_CID_CONTRAST;
  if (0 == ioctl (fd, VIDIOC_G_CTRL, &control)) 
  {
    printf("Setting contrast to (default=%d)\n", control.value + 1);
    control.value = value;
    if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control) && errno != ERANGE) 
    {
      perror ("VIDIOC_S_CTRL for V4L2_CID_CONTRAST");
    }
  } 
  else if (errno != EINVAL) 
  {
    printf("# V4L2_CID_CONTRAST failed\n");
    perror ("VIDIOC_G_CTRL for V4L2_CID_CONTRAST");
  }
  else
    printf("# V4L2_CID_CONTRAST not supported\n");
}  


////////////////////////////////////////////////////

void UV4l2::setGain(int gain)
{  //
  // gain auto (=0) or manual (>0)
  struct v4l2_control control;
  memset (&control, 0, sizeof (control));
  if (true)
  { // auto gain
    control.id = V4L2_CID_AUTOGAIN;
    if (0 == ioctl (fd, VIDIOC_G_CTRL, &control)) 
    {
      printf("Getting autogain is (%d)\n", control.value);
      control.value = gain <= 0;
      printf("Setting autogain to (%d)\n", control.value);
      
      if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control) && errno != ERANGE) 
      {
        perror ("VIDIOC_S_CTRL for V4L2_CID_AUTOGAIN");
      }
    } 
    else if (errno != EINVAL) 
    {
      printf("# V4L2_CID_AUTOGAIN failed\n");
      perror ("VIDIOC_G_CTRL for V4L2_CID_AUTOGAIN");
    }
    else
      printf("# V4L2_CID_AUTOGAIN not supported\n");
  }
  if (gain > 0)
  { // manual gain
    memset (&control, 0, sizeof (control));
    control.id = V4L2_CID_GAIN;    
    if (0 == ioctl (fd, VIDIOC_G_CTRL, &control)) 
    {
      printf("Getting gain is (%d)\n", control.value);
      control.value = gain;
      printf("Setting gain to (%d)\n", control.value);
      if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control) && errno != ERANGE) 
      {
        perror ("VIDIOC_S_CTRL for V4L2_CID_GAIN");
      }
    } 
    else if (errno != EINVAL) 
    {
      printf("# V4L2_CID_GAIN failed\n");
      perror ("VIDIOC_G_CTRL for V4L2_CID_GAIN");
      //     exit (EXIT_FAILURE);
    }
    else
      printf("# V4L2_CID_GAIN not supported\n");
  }
}
  

void UV4l2::setWhiteBalance(bool toAuto)
{ ////////////////////////////////////////////////////
  //
  // white balance auto
  //
  struct v4l2_control control;
  memset (&control, 0, sizeof (control));
  control.id = V4L2_CID_AUTO_WHITE_BALANCE;
  if (0 == ioctl (fd, VIDIOC_G_CTRL, &control)) 
  {
    printf("Getting V4L2_CID_AUTO_WHITE_BALANCE is (%d)\n", control.value);
    if (control.value != toAuto)
    {
      control.value = toAuto;
      printf("Setting V4L2_CID_AUTO_WHITE_BALANCE to (%d)\n", control.value);
      if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control) && errno != ERANGE) 
      {
        perror ("VIDIOC_S_CTRL for V4L2_CID_AUTO_WHITE_BALANCE");
      }
    }
  } 
  else if (errno != EINVAL) 
  {
    printf("# V4L2_CID_AUTO_WHITE_BALANCE failed\n");
  }
  else
    printf("# V4L2_CID_AUTO_WHITE_BALANCE not supported\n");
}

void UV4l2::setExposure(int value)
{
  ////////////////////////////////////////////////////
  //
  // white balance auto
  //
  struct v4l2_control control;
  memset (&control, 0, sizeof (control));
  control.id = V4L2_CID_EXPOSURE;
  if (0 == ioctl (fd, VIDIOC_G_CTRL, &control)) 
  {
    printf("Getting V4L2_CID_EXPOSURE is (%d)\n", control.value);
    if (control.value != value)
    {
      control.value = value;
      printf("Setting V4L2_CID_EXPOSURE to (%d)\n", control.value);
      if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control) && errno != ERANGE) 
      {
        perror ("VIDIOC_S_CTRL for V4L2_CID_EXPOSURE");
      }
    }
  } 
  else if (errno != EINVAL) 
  {
    printf("# V4L2_CID_EXPOSURE failed\n");
  }
  else
    printf("# V4L2_CID_EXPOSURE not supported\n");
}


const void UV4l2::usage(FILE *fp, int argc, char **argv)
{
  fprintf(fp,
          "Usage: %s [options]\n\n"
          "Version 1.3\n"
          "Options:\n"
          "-d | --device name   Video device name [%s]\n"
          "-h | --help          Print this message\n"
          "-m | --mmap          Use memory mapped buffers [default]\n"
          "-r | --read          Use read() calls\n"
          "-u | --userp         Use application allocated buffers\n"
          "-o | --output        Outputs stream to stdout\n"
          "-f | --format        Force format to 640x480 YUYV\n"
          "-c | --count         Number of frames to grab [%i]\n"
          "",
          argv[0], dev_name, frame_count);
}


int UV4l2::test_library()
{
  // dev_name = "/dev/video0";
  dev_name = "/dev/video0";
  io = IO_METHOD_MMAP;
  out_buf = 0; // no save
  w = 1920;
  h = 1080;
  //
//   w = 2592;
//   h = 1944;
  //
//   w = 640;
//   h = 480;
  //
  //pixelFormat = V4L2_PIX_FMT_YUYV;
  pixelFormat = V4L2_PIX_FMT_SBGGR10;
  //pixelFormat = V4L2_PIX_FMT_SBGGR10P;
  //pixelFormat = V4L2_PIX_FMT_SBGGR8;
  // pixelFormat = V4L2_PIX_FMT_RGB24;
  force_format = 1; // set w,h, ...
  frame_count = 300;
  //
  open_device();
  init_device();
  start_capturing();
  mainloop();
  stop_capturing();
  uninit_device();
  close_device();
  fprintf(stderr, "\n");
  return 0;
}


// int main(int argc, char *argv[])
// {
//     puts("Hello, World!");
//     return 0;
// }
