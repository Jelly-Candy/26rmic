#include "CameraModel.hpp"
#include "AimUart.hpp"
#include <opencv2/opencv.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
static int32_t i32(const uint8_t *b){uint32_t u=uint32_t(b[0])|(uint32_t(b[1])<<8)|(uint32_t(b[2])<<16)|(uint32_t(b[3])<<24);return int32_t(u);}
static uint16_t crc(const uint8_t *p,int n){uint16_t c=0;for(int i=0;i<n;i++){c^=p[i];for(int j=0;j<8;j++)c=(c>>1)^((c&1)?0xa001:0);}return c;}
int main(int argc,char **argv){assert(argc==2);CameraModel m;assert(m.load(argv[1]));assert(m.setImageSize(640,480));
 cv::Mat raw(480,640,CV_8UC3,cv::Scalar(0,0,0));raw.at<cv::Vec3b>(0,0)=cv::Vec3b(1,2,3);raw.at<cv::Vec3b>(479,639)=cv::Vec3b(7,8,9);cv::rotate(raw,raw,cv::ROTATE_180);assert(raw.at<cv::Vec3b>(479,639)==cv::Vec3b(1,2,3));assert(raw.at<cv::Vec3b>(0,0)==cv::Vec3b(7,8,9));
 auto z=m.mountedPrincipalPoint();auto a=m.mountedPixelToAngle(z);assert(std::abs(a.x)<1e-5 && std::abs(a.y)<1e-5);
 for(int dx:{-100,100})for(int dy:{-70,70}){auto p=z+cv::Point2f(dx,dy);auto v=m.mountedPixelToAngle(p);assert(v.x*dx>0 && v.y*dy<0);
 auto orig=m.pixelToAngle(cv::Point2f(639-p.x,479-p.y));assert(std::abs(v.x+orig.x)<1e-5 && std::abs(v.y+orig.y)<1e-5);
 AimResult result;result.targetFound=true;result.yawDeg=v.x;result.pitchDeg=v.y;result.imu.valid=true;result.imu.gy=12.345f;uint8_t f[AimUart::FRAME_SIZE];AimUart::encode(result,9,f);assert(AimUart::FRAME_SIZE==48 && f[0]==0xa5 && f[1]==0x5a && f[2]==2 && f[3]==9);assert(f[4]&1);assert(i32(f+6)==std::lround(v.x*1000) && i32(f+10)==std::lround(v.y*1000));assert(i32(f+30)==12345);assert(crc(f,46)==uint16_t(f[46]|(uint16_t(f[47])<<8)));
 }
 AimResult lost;uint8_t f[48];AimUart::encode(lost,0,f);assert(!(f[4]&1) && i32(f+6)==0 && i32(f+10)==0);
 puts("PASS: actual OpenCV 180-degree pixel rotation, asymmetric calibrated principal point, all four quadrants, raw/mounted ray equivalence, 48-byte signed angles/CRC, IMU field unchanged and target loss.");
}
