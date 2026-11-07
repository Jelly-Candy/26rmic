#include "ColorContourDetector.hpp"
#include "PixelMotionTracker.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <iostream>
static void require(bool yes,const char *why){if(!yes)throw std::runtime_error(why);}
int main(){
  ColorContourDetector detector(TargetColor::BLUE);PixelMotionTracker tracker;
  int measured=0,predicted=0;
  for(int i=0;i<60;++i){
    const int x=150+i*3;cv::Mat rgb(480,640,CV_8UC3,cv::Scalar(0,0,0));
    if(i!=25 && i!=26){
      cv::rectangle(rgb,{x-40,190,8,60},{20,245,252},cv::FILLED);
      cv::rectangle(rgb,{x+32,190,8,60},{20,245,252},cv::FILLED);
    }
    auto detections=detector.detect(rgb);const auto &t=tracker.update(detections,1./30);
    if(i==25 || i==26){require(detections.empty(),"occlusion should have no image detection");require(t.state==TrackState::TEMP_LOST,"short occlusion not visually predicted");++predicted;}
    else {require(detections.size()==1,"synthetic colored object not detected");require(t.state==TrackState::TRACKING,"visible object not tracked");require(std::abs(t.center.x-x)<6,"pixel trajectory mismatch");++measured;}
  }
  for(int i=0;i<9;++i)tracker.update({},1./30);
  require(tracker.getTarget().state==TrackState::LOST,"expired visual prediction not cleared");
  std::cout<<"PASS image->contour->pixel track: "<<measured<<" observed frames, "<<predicted<<" occluded frames, expiry\n";
}
