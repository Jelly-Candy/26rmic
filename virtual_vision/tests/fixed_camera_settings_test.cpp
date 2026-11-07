#include "FixedCameraSettings.hpp"
#include <cassert>
#include <iostream>
template<class F>void rejects(F f){bool failed=false;try{f();}catch(const std::runtime_error&){failed=true;}assert(failed);}
void env(const char*k,const char*v){
#ifdef _WIN32
 _putenv_s(k,v?v:"");
#else
 if(v)setenv(k,v,1);else unsetenv(k);
#endif
}
int main(){
 FixedCameraSettings s;
 std::array<uint32_t,12> good={0,7981,2048,1024,1024,7981,2048,1024,1024,8602,0,0};
 s.validate(good);s.validateHold(good,good);
 auto r=good;r[0]=1;rejects([&]{s.validate(r);});
 r=good;r[5]=10000;rejects([&]{s.validate(r);});
 for(int i:{2,3,4,6,7,8,10,11}){r=good;r[i]++;rejects([&]{s.validate(r);});}
 r=good;r[9]++;s.validate(r);rejects([&]{s.validateHold(good,r);});
 r=good;r[5]++;s.validate(r);rejects([&]{s.validateHold(good,r);});
 for(const char*v:{"-1","0","8000junk","1.5","99999999999999999999999","33334"}){
  env("RMIC_FIXED_EXPOSURE_US",v);rejects([]{FixedCameraSettings::environment();});
 }
 env("RMIC_FIXED_EXPOSURE_US","6000");assert(FixedCameraSettings::environment().exposureUs==6000);
 env("RMIC_FIXED_EXPOSURE_US",nullptr);
 env("RMIC_FIXED_ANALOG_GAIN_Q10","15591");assert(FixedCameraSettings::environment().analogGain==15591);
 env("RMIC_FIXED_ANALOG_GAIN_Q10",nullptr);
 assert(FixedCameraSettings::environment().exposureUs==8000);
 std::cout<<"PASS fixed settings: validation, quantization, gain stages, AE rejection, runtime drift, persisted-setting parsing\n";
}
