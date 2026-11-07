#include "PixelMotionTracker.hpp"
#include "LegacyPixelTracker.hpp"
#include <stdexcept>
#include <iostream>
#include <limits>
#include <chrono>
static void require(bool p,const char*s){if(!p)throw std::runtime_error(s);}
static VisualDetection det(float x,float y=180,int w=24,int h=24){VisualDetection d{};d.center={x,y};d.bbox={cvRound(x)-w/2,cvRound(y)-h/2,w,h};return d;}
int main(){
 LegacyPixelTracker old;PixelMotionTracker tracker;float x=80;old.update({det(x)},.05);tracker.update({det(x)},.05);double oldError=0,newError=0;
 for(int i=0;i<8;++i){x+=20;auto a=old.update({det(x)},.05),b=tracker.update({det(x)},.05);require(b.state==TrackState::TRACKING && b.detectionIndex==0,"startup match");oldError+=cv::norm(a.center-cv::Point2f(x,180));newError+=cv::norm(b.center-cv::Point2f(x,180));}
 require(newError<oldError*.35,"startup error did not improve");std::cout<<"STARTUP mean error old="<<oldError/8<<" new="<<newError/8<<" pixels\n";
 for(int i=0;i<6;++i){x-=20;const auto &b=tracker.update({det(x)},.05);require(b.state==TrackState::TRACKING && std::abs(b.center.x-x)<5,"abrupt direction change");}
 for(double dt:{.01,.05,.1}){PixelMotionTracker m;m.update({det(100)},dt);m.update({det(100)},dt);int n=0;double elapsed=0;while(elapsed+dt<=.25000001){auto a=m.update({},dt);elapsed+=dt;++n;require(a.state==TrackState::TEMP_LOST && a.detectionIndex==-1,"time based coast lost early");}require(m.update({},dt).state==TrackState::LOST,"coast did not expire");std::cout<<"COAST dt="<<dt<<" survived "<<n<<" missing frames, expired by elapsed time\n";}
 PixelMotionTracker gap;float px=100;gap.update({det(px)},.05);for(int i=0;i<10;++i){px+=5;gap.update({det(px)},.05);}px+=15;require(gap.update({det(px)},.15).state==TrackState::TRACKING,"150ms jitter reset");auto predicted=gap.update({},.10);require(predicted.state==TrackState::TEMP_LOST,"miss prediction");require(predicted.center.x>predicted.observedCenter.x+5,"prediction frozen");require(gap.update({det(px+15)},.05).state==TrackState::TRACKING,"reassociation");
 PixelMotionTracker crossing;crossing.update({det(80),det(440,180,20,20)},.05);int coast=0;
 for(int i=1;i<=44;++i){float a=80+6*i,b=440-6*i;std::vector<VisualDetection>ds=i%2?std::vector<VisualDetection>{det(b),det(a)}:std::vector<VisualDetection>{det(a),det(b)};auto t=crossing.update(ds,.05);require(t.state!=TrackState::LOST,"crossing lost");if(t.state==TrackState::TRACKING)require(std::abs(t.observedCenter.x-a)<1,"switched crossing target");else ++coast;}
 require(coast<=2,"excessive crossing ambiguity");std::cout<<"CROSSING coast="<<coast<<"; no wrong assignment in measured frames\n";

 PixelMotionTracker edge;for(int i=0;i<=4;++i)edge.update({det(540+20*i)},.05);require(edge.update({det(600)},.05).state==TrackState::TRACKING,"edge reversal ignored valid observation");
 PixelMotionTracker noisy;cv::RNG rng(20261106);double measuredSquared=0,filteredSquared=0;int measuredCount=0;
 for(int i=0;i<40;++i){cv::Point2f truth(100+5*i,220);auto observed=det(truth.x+float(rng.gaussian(2)),truth.y+float(rng.gaussian(2)));auto t=noisy.update({observed},.05);if(i>=10){require(t.state==TrackState::TRACKING,"noisy trajectory lost");measuredSquared+=cv::norm(observed.center-truth)*cv::norm(observed.center-truth);filteredSquared+=cv::norm(t.center-truth)*cv::norm(t.center-truth);++measuredCount;}}
 require(filteredSquared<measuredSquared,"steady noise not smoothed");std::cout<<"NOISY synthetic measured_RMSE="<<std::sqrt(measuredSquared/measuredCount)<<" filtered_RMSE="<<std::sqrt(filteredSquared/measuredCount)<<" pixels\n";
 PixelMotionTracker outlier;outlier.update({det(200)},.05);for(int i=0;i<8;++i)outlier.update({det(200)},.05);require(outlier.update({det(320)},.05).state==TrackState::TEMP_LOST,"outlier grabbed track");require(outlier.update({det(200)},.05).state==TrackState::TRACKING,"outlier return");auto invalid=det(210);invalid.center.x=std::numeric_limits<float>::quiet_NaN();require(outlier.update({invalid},.05).state==TrackState::TEMP_LOST,"NaN accepted");
 PixelMotionTracker flash;flash.update({det(200)},.05);require(flash.update({},.05).state==TrackState::LOST,"one-frame false detection buffered");
 PixelMotionTracker bad;require(bad.update({invalid},.05).state==TrackState::LOST,"invalid acquisition");auto malformed=det(200);malformed.bbox.width=std::numeric_limits<int>::max();require(bad.update({malformed},.05).state==TrackState::LOST,"bad box accepted");
 FrameTiming legacyClock,clock(.30);legacyClock.tickAt(1000000);clock.tickAt(1000000);require(legacyClock.tickAt(1150000).resetTracking,"legacy timing changed");require(clock.tickAt(1150000).valid,"motion timing rejects jitter");require(clock.tickAt(1451000).resetTracking,"long gap accepted");require(std::abs(clock.tickAt(1501000).rawDtSeconds-.05)<1e-9,"raw dt missing");
 PixelMotionTracker speed;auto begin=std::chrono::steady_clock::now();for(int i=0;i<10000;++i)speed.update({det(200+float(i%100)*.1f)},.01);auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();std::cout<<"BENCH host update_us="<<ms/10<<" (not camera FPS)\n";
 std::cout<<"PASS startup, reversal, time expiry, jitter, reacquisition, crossing, outliers, invalid input and pixel-coordinate tracking\n";
}
