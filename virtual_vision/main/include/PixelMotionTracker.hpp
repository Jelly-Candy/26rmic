#pragma once
// Image-space motion tracker preserved from the Nov06 input; no angle or control outputs.
#include "PixelTrack.hpp"
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

class PixelMotionTracker {
public:
  struct Diagnostic {
    const char *reason="EMPTY";
    double missingSeconds=0;
    float distance=0,mahalanobis=0;
    int candidates=0;
  };
  static constexpr double MAX_INTERVAL_SECONDS=0.300;
  static constexpr double MAX_COAST_SECONDS=0.250;
  const Diagnostic &diagnostic()const{return diagnostic_;}
  const TrackedTarget &getTarget()const{return target_;}
  PixelMotionTracker(){kf_.init(4,2,0,CV_32F);kf_.measurementMatrix=cv::Mat::zeros(2,4,CV_32F);kf_.measurementMatrix.at<float>(0,0)=1;kf_.measurementMatrix.at<float>(1,1)=1;}
  void reset(){target_=TrackedTarget{};age_=0;hits_=0;diagnostic_=Diagnostic{};}
  const TrackedTarget &update(const std::vector<VisualDetection>& detections,double dt=1.0/30){
    diagnostic_=Diagnostic{};
    if(!std::isfinite(dt) || dt<0.001 || dt>MAX_INTERVAL_SECONDS){reset();acquire(detections);diagnostic_.reason=target_.state==TrackState::TRACKING?"REACQUIRE_INVALID_DT":"INVALID_DT";return target_;}
    if(target_.state==TrackState::LOST)return acquire(detections);
    age_+=dt;
    if(age_>MAX_COAST_SECONDS+1e-9){reset();acquire(detections);diagnostic_.reason=target_.state==TrackState::TRACKING?"REACQUIRE_TIMEOUT":"TIMEOUT";return target_;}
    const float t=float(dt),t2=t*t,t3=t2*t,t4=t2*t2;
    const float acceleration=std::clamp(8.0f*std::max(target_.bbox.width,target_.bbox.height),300.0f,1000.0f);
    const float q=acceleration*acceleration;
    kf_.transitionMatrix=(cv::Mat_<float>(4,4)<<1,0,t,0,0,1,0,t,0,0,1,0,0,0,0,1);
    kf_.processNoiseCov=(cv::Mat_<float>(4,4)<<q*t4/4,0,q*t3/2,0,0,q*t4/4,0,q*t3/2,q*t3/2,0,q*t2,0,0,q*t3/2,0,q*t2);
    const cv::Mat predicted=kf_.predict();
    const cv::Point2f center(predicted.at<float>(0),predicted.at<float>(1));
    const cv::Rect predictedBox=cv::Rect(cvRound(center.x)-target_.bbox.width/2,cvRound(center.y)-target_.bbox.height/2,target_.bbox.width,target_.bbox.height);
    const double oldArea=double(target_.bbox.width)*target_.bbox.height;
    const float physicalGate=std::min(240.0f,std::max(25.0f,0.5f*std::hypot(float(target_.bbox.width),float(target_.bbox.height))+1800.0f*float(age_)));
    int best=-1,compatible=0,sole=-1;float bestScore=1e30f,second=1e30f,bestDistance=0,bestMaha=0;
    for(size_t i=0;i<detections.size();++i){const auto &d=detections[i];if(!valid(d))continue;
      const double area=double(d.bbox.width)*d.bbox.height;
      const float areaRatio=float(std::min(oldArea,area)/std::max(oldArea,area));
      const double aspect=(double(d.bbox.width)/d.bbox.height)/(double(target_.bbox.width)/target_.bbox.height);
      const auto delta=d.center-center;const float distance=cv::norm(delta);
      if(areaRatio<0.35f || aspect<1.0/3 || aspect>3 || distance>physicalGate)continue;
      ++compatible;sole=int(i);
      const float variance=measurementVariance(d);
      const float a=kf_.errorCovPre.at<float>(0,0)+variance,b=kf_.errorCovPre.at<float>(0,1),c=kf_.errorCovPre.at<float>(1,1)+variance;
      const float determinant=a*c-b*b;if(determinant<=1e-6f)continue;
      const float maha=(c*delta.x*delta.x-2*b*delta.x*delta.y+a*delta.y*delta.y)/determinant;
      if(!std::isfinite(maha) || maha>9.21034f)continue; // 99% chi-square, 2 positional DOF
      const float score=maha+2*(1-iou(predictedBox,d.bbox))+2*(1-areaRatio);
      if(score<bestScore){second=bestScore;bestScore=score;best=int(i);bestDistance=distance;bestMaha=maha;}
      else second=std::min(second,score);
    }
    diagnostic_.candidates=compatible;
    bool maneuver=false;
    // Bounded unique-observation recovery for abrupt turns. Require overlap
    // with the LAST REAL box or a bounded size/speed neighbourhood.
    if(best<0 && compatible==1 && age_<=0.120 && sole>=0){
      const auto &d=detections[sole];const double area=double(d.bbox.width)*d.bbox.height;
      const float ratio=float(std::min(oldArea,area)/std::max(oldArea,area));
      const float radius=std::min(80.0f,std::max(0.75f*std::hypot(float(target_.bbox.width),float(target_.bbox.height)),1.25f*float(cv::norm(target_.velocity))*float(age_)));
      if(ratio>=0.70f && (iou(lastRealBox_,d.bbox)>=0.10f || cv::norm(d.center-target_.observedCenter)<=radius) && cv::norm(d.center-target_.observedCenter)<=1800.0f*float(age_)){
        best=sole;maneuver=true;bestDistance=cv::norm(d.center-center);bestMaha=std::numeric_limits<float>::quiet_NaN();
      }
    }
    if(best>=0 && !maneuver && second-bestScore<0.25f){best=-1;diagnostic_.reason="AMBIGUOUS";}
    if(best>=0){const auto &d=detections[best];
      const auto previousObserved=target_.observedCenter;const double observedInterval=age_;
      cv::setIdentity(kf_.measurementNoiseCov,cv::Scalar::all(measurementVariance(d)));
      if(maneuver){kf_.statePre.at<float>(0)=d.center.x;kf_.statePre.at<float>(1)=d.center.y;
        kf_.statePre.at<float>(2)=float((d.center.x-previousObserved.x)/observedInterval);
        kf_.statePre.at<float>(3)=float((d.center.y-previousObserved.y)/observedInterval);
        kf_.errorCovPre.at<float>(2,2)=360000;kf_.errorCovPre.at<float>(3,3)=360000;}
      const cv::Mat corrected=kf_.correct((cv::Mat_<float>(2,1)<<d.center.x,d.center.y));
      target_.center={corrected.at<float>(0),corrected.at<float>(1)};target_.velocity={corrected.at<float>(2),corrected.at<float>(3)};
      target_.observedCenter=d.center;target_.bbox=d.bbox;lastRealBox_=d.bbox;target_.detectionIndex=best;target_.missedFrames=0;target_.state=TrackState::TRACKING;age_=0;hits_=std::min(2,hits_+1);
      diagnostic_.reason=maneuver?"MANEUVER_MATCH":"MATCH";diagnostic_.distance=bestDistance;diagnostic_.mahalanobis=bestMaha;return target_;
    }
    if(!inside(center)){reset();diagnostic_.reason="OUT_OF_VIEW";return target_;}
    if(hits_<2){reset();diagnostic_.reason="UNCONFIRMED_MISS";return target_;}
    kf_.statePost=kf_.statePre.clone();kf_.errorCovPost=kf_.errorCovPre.clone();
    target_.center=center;target_.velocity={predicted.at<float>(2),predicted.at<float>(3)};target_.bbox=predictedBox;
    target_.state=TrackState::TEMP_LOST;target_.detectionIndex=-1;++target_.missedFrames;diagnostic_.missingSeconds=age_;
    if(diagnostic_.reason!=std::string("AMBIGUOUS"))diagnostic_.reason=detections.empty()?"NO_DETECTION":"NO_MATCH";
    return target_;
  }
private:
  cv::KalmanFilter kf_;
  TrackedTarget target_;
  cv::Rect lastRealBox_;
  double age_=0;
  int hits_=0;
  Diagnostic diagnostic_;
  static bool inside(cv::Point2f p){return std::isfinite(p.x)&&std::isfinite(p.y)&&p.x>=0&&p.y>=0&&p.x<640&&p.y<480;}
  static bool valid(const VisualDetection &d){return inside(d.center)&&d.bbox.width>0&&d.bbox.height>0&&d.bbox.width<=640&&d.bbox.height<=480&&d.bbox.x>=0&&d.bbox.y>=0&&d.bbox.x<=640-d.bbox.width&&d.bbox.y<=480-d.bbox.height;}
  static float measurementVariance(const VisualDetection &d){const float sigma=std::clamp(0.05f*std::sqrt(float(d.bbox.width)*d.bbox.height),2.0f,8.0f);return sigma*sigma;}
  static float iou(const cv::Rect&a,const cv::Rect&b){const double intersection=(a&b).area();const double total=double(a.width)*a.height+double(b.width)*b.height-intersection;return total>0?float(intersection/total):0;}
  const TrackedTarget &acquire(const std::vector<VisualDetection>& detections){
    int best=-1;double largest=0;for(size_t i=0;i<detections.size();++i)if(valid(detections[i])){const double a=double(detections[i].bbox.width)*detections[i].bbox.height;if(a>largest){largest=a;best=int(i);}}
    if(best<0)return target_;
    const auto &d=detections[best];target_=TrackedTarget{};target_.center=target_.observedCenter=d.center;target_.bbox=lastRealBox_=d.bbox;target_.state=TrackState::TRACKING;target_.detectionIndex=best;age_=0;hits_=1;
    kf_.statePost=(cv::Mat_<float>(4,1)<<d.center.x,d.center.y,0,0);kf_.statePre=kf_.statePost.clone();
    kf_.errorCovPost=cv::Mat::zeros(4,4,CV_32F);kf_.errorCovPost.at<float>(0,0)=measurementVariance(d);kf_.errorCovPost.at<float>(1,1)=measurementVariance(d);kf_.errorCovPost.at<float>(2,2)=360000;kf_.errorCovPost.at<float>(3,3)=360000;kf_.errorCovPre=kf_.errorCovPost.clone();
    diagnostic_.reason="ACQUIRE";return target_;
  }
};
