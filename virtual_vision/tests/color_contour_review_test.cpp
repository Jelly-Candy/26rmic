#include "ColorContourDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>

int main() {
  ColorContourDetector detector(TargetColor::BLUE);ContourDebugInfo debug;
  cv::Mat rgb=cv::Mat::zeros(240,320,CV_8UC3);
  // One continuous coloured panel, with a cyan core on its left and a pure
  // blue edge on its right, must not become a fictitious pair of lamps.
  cv::rectangle(rgb,{70,80,50,60},{100,180,200},cv::FILLED);
  cv::rectangle(rgb,{70,80,10,60},{20,245,252},cv::FILLED);
  cv::rectangle(rgb,{110,80,10,60},{0,0,255},cv::FILLED);
  assert(detector.detect(rgb,&debug).empty());
  rgb.setTo(0);
  cv::rectangle(rgb,{0,80,10,60},{20,245,252},cv::FILLED);
  cv::rectangle(rgb,{120,80,10,60},{20,245,252},cv::FILLED);
  assert(detector.detect(rgb,&debug).size()==1);
  cv::Mat blueBackground(240,320,CV_8UC3,cv::Scalar(44,45,120));
  cv::rectangle(blueBackground,{0,80,10,60},{20,245,252},cv::FILLED);
  cv::rectangle(blueBackground,{120,80,10,60},{20,245,252},cv::FILLED);
  assert(detector.detect(blueBackground).size()==1); // Dark blue illumination is allowed.
  for(const auto &background : {cv::Scalar(120,120,120), cv::Scalar(160,0,0),
                              cv::Scalar(0,160,0), cv::Scalar(0,0,255)}) {
    cv::Mat scene(240,320,CV_8UC3,background);
    cv::rectangle(scene,{0,80,10,60},{20,245,252},cv::FILLED);
    cv::rectangle(scene,{120,80,10,60},{20,245,252},cv::FILLED);
    assert(detector.detect(scene).empty());
  }
  cv::Mat dark(240,320,CV_8UC3,cv::Scalar(35,35,35));
  cv::rectangle(dark,{0,80,10,60},{20,245,252},cv::FILLED);
  cv::rectangle(dark,{120,80,10,60},{20,245,252},cv::FILLED);
  assert(detector.detect(dark).size()==1);
  // A third lamp between the outer pair must not be spanned.
  cv::rectangle(dark,{60,80,10,60},{20,245,252},cv::FILLED);
  for(const auto &target:detector.detect(dark)) assert(target.bbox.width<140);
  cv::Mat blueSpill(240,320,CV_8UC3,cv::Scalar(60,70,220));
  cv::rectangle(blueSpill,{0,80,10,60},{20,245,252},cv::FILLED);
  cv::rectangle(blueSpill,{120,80,10,60},{20,245,252},cv::FILLED);
  assert(detector.detect(blueSpill).size()==1);
  // Bright structure between adjacent lamps must still reject their pairing.
  cv::rectangle(blueSpill,{45,80,40,60},{170,130,220},cv::FILLED);
  assert(detector.detect(blueSpill).empty());
  cv::Mat storage=cv::Mat::zeros(260,350,CV_8UC3);
  cv::Mat roi=storage(cv::Rect(5,5,320,240));rgb.copyTo(roi);
  assert(!roi.isContinuous());
  assert(detector.detect(roi).size()==1);
  detector.detect(cv::Mat{},&debug);
  assert(debug.mask.empty() && debug.lights.empty() && debug.diagnostics.empty() && debug.pairs.empty() && debug.pairScores.empty() && debug.keptPairs.empty());
  detector.detect(rgb,&debug);
  detector.detect(cv::Mat::zeros(240,320,CV_8UC1),&debug);
  assert(debug.mask.empty() && debug.lights.empty() && debug.diagnostics.empty());
  cv::Mat narrow=cv::Mat::zeros(240,320,CV_8UC3);
  cv::rectangle(narrow,{60,80,3,29},{20,245,252},cv::FILLED);
  cv::rectangle(narrow,{115,80,3,29},{20,245,252},cv::FILLED);
  assert(detector.detect(narrow).size()==1);
  cv::Mat noise(480,640,CV_8UC3);cv::RNG rng(260030);rng.fill(noise,cv::RNG::UNIFORM,0,256);
  detector.detect(noise,&debug);
  unsigned tiny=0;for(const auto &item:debug.diagnostics)if(std::string(item.source)=="CYAN_CORE" && item.area<20)++tiny;
  std::cout<<"NOISE tiny_core_contours="<<tiny<<" all_contours="<<debug.diagnostics.size()<<"\n";
  assert(tiny<1000);
  std::cout<<"PASS continuous coloured-panel rejection, border lamps, non-contiguous RGB input and stale-debug clearing\n";
}
