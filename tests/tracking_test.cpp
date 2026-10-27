#include "AimTrackGate.hpp"
#include <opencv2/opencv.hpp>
#include <cassert>
#include <cstdio>
static cv::Mat board(cv::Scalar color,bool core=false){cv::Mat rgb(240,400,CV_8UC3,cv::Scalar(15,15,15));for(int x:{100,220}){cv::rectangle(rgb,cv::Rect(x,80,12,60),color,cv::FILLED);if(core)cv::rectangle(rgb,cv::Rect(x+3,83,6,54),cv::Scalar(255,245,245),cv::FILLED);}return rgb;}
int main(){ArmorDetector red(EnemyColor::RED),blue(EnemyColor::BLUE);
 assert(!red.detect(board(cv::Scalar(220,30,30))).empty());
 assert(!blue.detect(board(cv::Scalar(30,30,220))).empty());
 assert(red.detect(board(cv::Scalar(30,30,220))).empty());
 assert(blue.detect(board(cv::Scalar(220,30,30))).empty());
 assert(red.detect(board(cv::Scalar(255,255,255))).empty());
 assert(blue.detect(board(cv::Scalar(255,255,255))).empty());
 assert(!red.detect(board(cv::Scalar(240,25,25),true)).empty());
 AimTrackGate gate;TrackedTarget t;t.state=TrackState::TRACKING;t.center={120,80};cv::Point2f c;
 assert(gate.update(t,100,c) && !gate.held());t.state=TrackState::TEMP_LOST;t.missedFrames=1;assert(!gate.update(t,120,c));
 t.state=TrackState::TRACKING;assert(gate.update(t,200,c));assert(gate.update(t,230,c));
 t.state=TrackState::TEMP_LOST;t.center={999,999};t.missedFrames=1;assert(gate.update(t,250,c)&&gate.held()&&c==cv::Point2f(120,80));t.missedFrames=2;assert(gate.update(t,280,c));assert(!gate.update(t,291,c));
 t.state=TrackState::TRACKING;t.center={120,80};assert(gate.update(t,300,c));assert(gate.update(t,330,c));t.state=TrackState::TEMP_LOST;t.missedFrames=3;assert(!gate.update(t,340,c));
 t.state=TrackState::TRACKING;gate=AimTrackGate{};assert(gate.update(t,0xfffffff0U,c));assert(gate.update(t,0xfffffffaU,c));t.state=TrackState::TEMP_LOST;t.missedFrames=1;assert(gate.update(t,10,c));t.state=TrackState::LOST;assert(!gate.update(t,11,c));
 puts("PASS: red/blue saturated lamps, opposite-color/white rejection, white-core colored-fringe; isolated blink, two-frame/60ms hold, frozen center, true loss and tick rollover.");
}
