#pragma once
#include <array>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

struct FixedCameraSettings {
  int exposureUs=8000, analogGain=2048, digitalGain=1024, ispGain=1024, hcg=0;
  static int setting(const char *key,int fallback,int low,int high) {
    const char *v=std::getenv(key); if(!v) return fallback;
    errno=0;char *end=nullptr;const long n=std::strtol(v,&end,10);
    if(errno || end==v || *end || n<low || n>high)
      throw std::runtime_error(std::string("Invalid fixed camera setting: ")+key);
    return static_cast<int>(n);
  }
  static FixedCameraSettings environment() {
    FixedCameraSettings s;
    s.exposureUs=setting("RMIC_FIXED_EXPOSURE_US",8000,100,33333);
    s.analogGain=setting("RMIC_FIXED_ANALOG_GAIN_Q10",2048,1024,16384);
    s.digitalGain=setting("RMIC_FIXED_DIGITAL_GAIN_Q10",1024,1024,16384);
    s.ispGain=setting("RMIC_FIXED_ISP_GAIN_Q10",1024,1024,16384);
    s.hcg=setting("RMIC_FIXED_HCG_LCG",0,0,1);
    return s;
  }
  // Layout exported by native/fixed_ae.c: actual values come from GetAeStatus,
  // rather than the SDK's gain setter return value (a different gain stage).
  void validate(const std::array<uint32_t,12>& r) const {
    if(r[0]!=0) throw std::runtime_error("Camera auto exposure is enabled");
    if(std::abs(double(r[5])-exposureUs)>std::max(30.,exposureUs*.01))
      throw std::runtime_error("Actual exposure differs from saved request");
    if(r[2]!=unsigned(analogGain) || r[6]!=unsigned(analogGain) ||
       r[3]!=unsigned(digitalGain) || r[7]!=unsigned(digitalGain) ||
       r[4]!=unsigned(ispGain) || r[8]!=unsigned(ispGain) ||
       r[10]!=unsigned(hcg) || r[11]!=unsigned(hcg))
      throw std::runtime_error("Actual gain differs from saved request");
  }
  static void validateHold(const std::array<uint32_t,12>& before,
                           const std::array<uint32_t,12>& after) {
    for(int i=5;i<12;++i)
      if(before[i]!=after[i]) throw std::runtime_error("Fixed camera readback changed during runtime");
  }
};
