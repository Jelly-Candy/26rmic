#include "maix_app.hpp"
#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_sys.hpp"
#include "maix_time.hpp"
#include "ColorContourDetector.hpp"
#include "PixelMotionTracker.hpp"
#include "FrameTiming.hpp"
#include "FixedCameraExposure.hpp"
#include "LatestPreview.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

using namespace maix;
struct TrailPoint { cv::Point2f point; bool predicted; };
static const char *stateName(TrackState s) {
  return s==TrackState::TRACKING?"MEASURED":s==TrackState::TEMP_LOST?"PREDICTED":"LOST";
}
static void fixedLog(const char *kind,const std::array<uint32_t,12>& r) {
  std::printf("[%s] actual_us=%u analog_q10=%u digital_q10=%u isp_q10=%u total_q10=%u hcg=%u ae=%u\n",
    kind,r[5],r[6],r[7],r[8],r[9],r[11],r[0]);std::fflush(stdout);
}
static void dashed(cv::Mat &rgb,cv::Point2f a,cv::Point2f b,cv::Scalar color) {
  const float distance=cv::norm(b-a);
  if(distance<1)return;
  for(float t=0;t<distance;t+=8)
    cv::line(rgb,a+(b-a)*(t/distance),a+(b-a)*(std::min(t+4,distance)/distance),color,1,cv::LINE_AA);
}
static int run(int argc,char **argv) {
  double seconds=0;
  if(argc==3 && std::string(argv[1])=="--seconds") {
    char *end=nullptr;seconds=std::strtod(argv[2],&end);
    if(end==argv[2] || *end || !std::isfinite(seconds) || seconds<=0 || seconds>3600)
      throw std::runtime_error("--seconds must be a number in (0,3600]");
  } else if(argc!=1)throw std::runtime_error("Usage: VirtualVision [--seconds N]");
  const std::string color=std::getenv("VISION_TARGET_COLOR")?std::getenv("VISION_TARGET_COLOR"):"blue";
  if(color!="blue" && color!="red")throw std::runtime_error("VISION_TARGET_COLOR must be blue or red");
  const int previewFps=FixedCameraSettings::setting("VISION_PREVIEW_FPS",30,1,30);
  const int rotation=FixedCameraSettings::setting("VISION_ROTATION",180,0,180);
  if(rotation!=0 && rotation!=180)throw std::runtime_error("VISION_ROTATION must be 0 or 180");
  const int trailLength=FixedCameraSettings::setting("VISION_TRAIL_LENGTH",90,2,300);
  const std::string logDir=std::getenv("VISION_LOG_DIR")?std::getenv("VISION_LOG_DIR"):"../../logs";
  camera::Camera cam(640,480,image::Format::FMT_RGB888,nullptr,30);
  FixedCameraExposure fixed;fixed.configure(cam);fixedLog("FIXED_CAMERA",fixed.reference);
  ColorContourDetector detector(color=="blue"?TargetColor::BLUE:TargetColor::RED);
  PixelMotionTracker tracker;
  FrameTiming timing(PixelMotionTracker::MAX_INTERVAL_SECONDS);
  std::deque<TrailPoint> trail;
  LatestPreview<image::Image> sender([](image::Image &frame){display::send_to_maixvision(frame);});
  std::ofstream trace;
  if(const char *p=std::getenv("VISION_TRACE_CSV")) {
    trace.open(p);
    if(!trace)throw std::runtime_error("Cannot open pixel trajectory CSV");
    trace<<"capture_us,dt_ms,detections,track_state,observed_x_px,observed_y_px,filtered_x_px,filtered_y_px,vx_px_s,vy_px_s,missing_ms,association\n";
  }
  const uint64_t start=time::ticks_us();
  uint64_t lastReport=start,lastPreview=0,windowFrames=0,lastSent=0,totalFrames=0,totalDetected=0;
  double processingFps=0,sendFps=0;
  std::cout<<"[VIRTUAL_VISION] pixel boxes/trajectories only; color="<<color
           <<" preview_target="<<previewFps<<" capture=640x480@30; no pose or control modules\n";
  while(!app::need_exit() && (!seconds || (time::ticks_us()-start)*1e-6<seconds)) {
    sender.rethrowFailure();
    std::unique_ptr<image::Image> frame(cam.read(true,1000));
    if(!frame)throw std::runtime_error("Camera frame timeout");
    if(frame->format()!=image::Format::FMT_RGB888)throw std::runtime_error("Unexpected camera format");
    const uint64_t capture=time::ticks_us();
    const auto dt=timing.tickAt(capture);
    if(dt.resetTracking){tracker.reset();trail.clear();}
    cv::Mat rgb(frame->height(),frame->width(),CV_8UC3,frame->data());
    if(rotation==180)cv::rotate(rgb,rgb,cv::ROTATE_180);
    // Opt-in raw diagnostic frame, captured before preview overlays.
    if(const char *dir=std::getenv("VISION_DIAGNOSTIC_DIR")) {
      static uint64_t lastDump=0;
      if(capture-lastDump>=1000000) {
        lastDump=capture;
        const std::string path=std::string(dir)+"/latest.ppm";
        std::ofstream dump(path+".tmp",std::ios::binary);
        dump<<"P6\n"<<rgb.cols<<" "<<rgb.rows<<"\n255\n";
        for(int row=0;row<rgb.rows;++row)
          dump.write(reinterpret_cast<const char *>(rgb.ptr(row)),rgb.cols*3);
        dump.close();
        if(dump)std::rename((path+".tmp").c_str(),path.c_str());
      }
    }
    const auto detections=detector.detect(rgb);
    const auto previousState=tracker.getTarget().state;
    const auto &track=tracker.update(detections,dt.dtSeconds);
    const auto &diagnostic=tracker.diagnostic();
    ++totalFrames;++windowFrames;if(!detections.empty())++totalDetected;
    if(track.state==TrackState::LOST)trail.clear();
    else {
      if(previousState==TrackState::LOST || std::string(diagnostic.reason).find("REACQUIRE")==0)trail.clear();
      trail.push_back(TrailPoint{track.center,track.state==TrackState::TEMP_LOST});
      while(trail.size()>static_cast<size_t>(trailLength))trail.pop_front();
    }
    if(trace)trace<<capture<<','<<dt.rawDtSeconds*1000<<','<<detections.size()<<','<<stateName(track.state)<<','
      <<track.observedCenter.x<<','<<track.observedCenter.y<<','<<track.center.x<<','<<track.center.y<<','
      <<track.velocity.x<<','<<track.velocity.y<<','<<diagnostic.missingSeconds*1000<<','<<diagnostic.reason<<'\n';
    const bool render=previewFps==30 || !lastPreview || capture-lastPreview>=1000000ULL/previewFps;
    if(render) {
      lastPreview=capture;
      for(const auto &d:detections)cv::rectangle(rgb,d.bbox,cv::Scalar(0,255,0),2);
      for(size_t i=1;i<trail.size();++i) {
        if(trail[i].predicted || trail[i-1].predicted)dashed(rgb,trail[i-1].point,trail[i].point,cv::Scalar(255,165,0));
        else cv::line(rgb,trail[i-1].point,trail[i].point,cv::Scalar(0,255,255),2,cv::LINE_AA);
      }
      if(track.state!=TrackState::LOST) {
        const auto box=track.bbox & cv::Rect(0,0,640,480);
        if(!box.empty())cv::rectangle(rgb,box,track.state==TrackState::TRACKING?cv::Scalar(255,255,0):cv::Scalar(255,165,0),2);
        cv::circle(rgb,track.center,3,cv::Scalar(255,255,0),cv::FILLED);
      }
      cv::rectangle(rgb,{0,0,640,78},cv::Scalar(12,12,12),cv::FILLED);
      cv::putText(rgb,"VIRTUAL GAME VISION  |  PIXEL TRACKS",{10,21},cv::FONT_HERSHEY_SIMPLEX,.55,{0,255,255},1);
      const std::string state="Objects: "+std::to_string(detections.size())+"  Track: "+stateName(track.state);
      cv::putText(rgb,state,{10,43},cv::FONT_HERSHEY_SIMPLEX,.52,{255,255,255},1);
      char label[180];std::snprintf(label,sizeof(label),"FPS %.1f  PREVIEW %.1f  EXP %.3fms  A %.1fx  JPEG 95",
        processingFps,sendFps,fixed.reference[5]/1000.,fixed.reference[6]/1024.);
      cv::putText(rgb,label,{10,64},cv::FONT_HERSHEY_SIMPLEX,.43,{255,255,255},1);
      sender.submit(std::unique_ptr<image::Image>(frame->copy()));
    }
    const uint64_t now=time::ticks_us();
    if(now-lastReport>=1000000) {
      const auto actual=fixed.verify();fixedLog("FIXED_HOLD",actual);
      const double elapsed=(now-lastReport)*1e-6;const uint64_t sent=sender.completed();
      processingFps=windowFrames/elapsed;sendFps=(sent-lastSent)/elapsed;
      std::printf("[VISION] fps=%.2f preview=%.2f objects=%zu track=%s missing_ms=%.1f association=%s frames=%llu\n",
        processingFps,sendFps,detections.size(),stateName(track.state),diagnostic.missingSeconds*1000,diagnostic.reason,
        static_cast<unsigned long long>(totalFrames));std::fflush(stdout);
      const auto tmp=logDir+"/status.json.tmp",status=logDir+"/status.json";
      std::ofstream out(tmp);out<<"{\"ok\":true,\"capture_us\":"<<capture<<",\"frames\":"<<totalFrames
        <<",\"processing_fps\":"<<processingFps<<",\"preview_send_fps\":"<<sendFps
        <<",\"exposure_us\":"<<actual[5]<<",\"analog_gain_q10\":"<<actual[6]<<",\"digital_gain_q10\":"<<actual[7]
        <<",\"isp_gain_q10\":"<<actual[8]<<",\"total_gain_q10\":"<<actual[9]<<",\"ae\":"<<actual[0]
        <<",\"objects\":"<<detections.size()<<",\"track_state\":\""<<stateName(track.state)<<"\"}\n";
      out.close();if(!out || std::rename(tmp.c_str(),status.c_str())!=0)throw std::runtime_error("Cannot write runtime status");
      if(trace)trace.flush();
      lastReport=now;windowFrames=0;lastSent=sent;
    }
  }
  sender.stop();sender.rethrowFailure();
  std::printf("[SUMMARY] frames=%llu frames_with_detections=%llu preview_sent=%llu elapsed_s=%.3f\n",
    static_cast<unsigned long long>(totalFrames),static_cast<unsigned long long>(totalDetected),
    static_cast<unsigned long long>(sender.completed()),(time::ticks_us()-start)*1e-6);
  return 0;
}
int main(int argc,char **argv) {
  sys::register_default_signal_handle();
  try{return run(argc,argv);}
  catch(const std::exception &e){std::fprintf(stderr,"[ERROR] %s\n",e.what());return 1;}
}
