
#include <math.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>

#include "CImg.h"
#include "opencv2/core.hpp"

using namespace std;
using namespace cimg_library;

#include "CimgStereo.h"
#include "CimgStereoDisplay.h"
#include "smartdisplay3.h"

// Stereo Pairs are received in single images where upper half is left and lowe half is right.
smartdisplay3<unsigned char> disp_rectified;
// smartdisplay3<float> disp_disparity;
// smartdisplay3<float> disp_depth;
smartdisplay3<float> disp_X;
smartdisplay3<float> disp_Y;
smartdisplay3<unsigned char> disp_rectified_hor;
smartdisplay3<unsigned char> disp_colorDepth;

int main(int argc, char **argv) {
   int nf;

   // const char *filename = cimg_option("-i", (char*)0, "Input  Filename.");

   const char *filename = argv[1];
   bool showDisparity = 1;
   bool showX = 0;
   bool showY = 0;
   bool showDepth = 1;
   bool showColorAlign = 0;
   bool showHorizAlign = 0;
   bool showVertAlign = 1;
   bool showColorDepth = 0;
   bool showStereoDisplay = 0;

   bool saveRectifiedUpDown = 0;
   bool saveRectifiedLeftRight = 0;
   bool saveRectifiedSeparate = 0;

   bool savexyz = 0;

   char *c=0;

   c = getenv("SHOW_DISPARITY");
   if (c) showDisparity = atoi(c);

   c = getenv("SHOW_X");
   if (c) showX = atoi(c);

   c = getenv("SHOW_Y");
   if (c) showY = atoi(c);

   c = getenv("SHOW_DEPTH");
   if (c) showDepth = atoi(c);

   c = getenv("SHOW_COLORALIGN");
   if (c) showColorAlign = atoi(c);

   c = getenv("SHOW_HORIZALIGN");
   if (c) showHorizAlign = atoi(c);

   c = getenv("SHOW_VERTALIGN");
   if (c) showVertAlign = atoi(c);

   c = getenv("SHOW_COLORDEPTH");
   if (c) showColorDepth = atoi(c);



   c = getenv("SAVE_RECTIFIED_UPDOWN");
   if (c) saveRectifiedUpDown = atoi(c);

   c = getenv("SAVE_RECTIFIED_LEFTRIGHT");
   if (c) saveRectifiedLeftRight = atoi(c);

   c = getenv("SAVE_RECTIFIED_SEPARATE");
   if (c) saveRectifiedSeparate = atoi(c);

   CimgStereo cimgstereo;
   cimgstereo.read_config("stereo_config.xml");

   CimgStereoDisplay stereoDisplay(&cimgstereo);

   CImg<unsigned char> composite(filename);
   //composite.display("Input");

   CImg<float> xyz = cimgstereo.composite2xyz(composite).get_shared();

   // Access some of the internal information used WITHOUT copy (for free)
   CImg<unsigned char> rectLeft, rectRight;
   CImg<float> disparity;
   CImg<float> depth;
   cimgstereo.getLeftRectifiedGray(rectLeft);
   cimgstereo.getRightRectifiedGray(rectRight);
   cimgstereo.getDisparity(disparity);
   depth = xyz.get_shared_slice(2);

   if (saveRectifiedUpDown) {
      CImg<unsigned char> rectified = rectLeft.get_append(rectRight, 'y');
      rectified.save("rectifiedUD.png");
   }

   if (saveRectifiedLeftRight) {
      CImg<unsigned char> rectified = rectLeft.get_append(rectRight, 'x');
      rectified.save("rectifiedLR.png");
   }

   if (saveRectifiedSeparate) {
      rectLeft.save("rectifiedL.png");
      rectRight.save("rectifiedR.png");
   }

   if (savexyz) {
      CImg<unsigned char> color2;
      cimgstereo.getLeftRectified(color2);
      CImg<float> color2f(color2);
      color2f.append(xyz, 'c');

      color2f.save_cimg("xyzrgb.cimg");
      std::cout << "xyzrgb.cimg saved\n";
   }

   if (showColorAlign) {
      CImg<unsigned char> rectified = rectLeft.get_append(rectRight, 'c').append(rectRight, 'c');
      std::string title = "Rectified";
      rectified.display(title.c_str());
   }

   if (showHorizAlign) {
      CImg<unsigned char> rectified = rectLeft.get_append(rectRight, 'x');
      int w_1 = rectified.width() - 1;
      int h_10 = rectified.height() / 10;
      unsigned char black[] = {0, 0, 0};
      for (int k = 1; k < 9; k++) {
         rectified.draw_line(0, k * h_10, w_1, k * h_10, black, 0.7);
      }

      std::string title = "Rectified";
      rectified.display(title.c_str());
   }

   if(showVertAlign){
      CImg<unsigned char> rectified = rectLeft.get_append(rectRight, 'y');
      rectified.append(rectRight,'x');
      rectified.display("Rectificadas");
   }

   if (showColorDepth) {
      std::string title2 = "Color Depth";
      CImg<float> tmp ;
      cimgstereo.depthAsColor(tmp);
      tmp.display(title2.c_str(), false);
   }

   if (showDisparity) {
      disparity.display("Disparity", false);
   }

   if (showDepth) {
      std::string title3 = "Depth";
      cimg_foroff(depth, o) {
         if (isnan(depth[o]))
            depth[o] = 0.0;
      }
      depth.display(title3.c_str(), false);
   }

   if (showX) {
      CImg<float> X = xyz.get_shared_slice(0);
      std::string title3 = "X-coord";
      cimg_foroff(X, o) {
         if (isnan(X[o]))
            X[o] = 0.0;
      }
      disp_X.display(X, title3);
   }
   if (showY) {
      CImg<float> Y = xyz.get_shared_slice(1);
      std::string title3 = "Y-coord";
      cimg_foroff(Y, o) {
         if (isnan(Y[o]))
            Y[o] = 0.0;
      }
      disp_Y.display(Y, title3);
   }

   std::cout << "ShowDisparity:" << showDisparity << "\n";
}
