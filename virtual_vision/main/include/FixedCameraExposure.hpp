#pragma once
#include "FixedCameraSettings.hpp"
#include <dlfcn.h>

class FixedCameraExposure {
  void *library=nullptr;
  using Apply=int (*)(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
  using Read=int (*)(uint32_t*);
  Apply apply=nullptr;Read read=nullptr;
public:
  FixedCameraSettings settings;
  std::array<uint32_t,12> reference{};
  FixedCameraExposure():settings(FixedCameraSettings::environment()) {
    library=dlopen("./libfixed_ae.so",RTLD_NOW|RTLD_LOCAL);
    if(!library) throw std::runtime_error(std::string("Fixed exposure library: ")+dlerror());
    apply=reinterpret_cast<Apply>(dlsym(library,"fixed_apply"));
    read=reinterpret_cast<Read>(dlsym(library,"fixed_read"));
    if(!apply || !read) {dlclose(library);library=nullptr;throw std::runtime_error("Fixed exposure library exports missing");}
  }
  ~FixedCameraExposure(){if(library)dlclose(library);}
  FixedCameraExposure(const FixedCameraExposure&)=delete;
  FixedCameraExposure& operator=(const FixedCameraExposure&)=delete;
  std::array<uint32_t,12> readback() const {
    std::array<uint32_t,12> out{};
    if(read(out.data())!=0)throw std::runtime_error("ISP exposure/gain status read failed");
    settings.validate(out);return out;
  }
  template<class Camera> void configure(Camera& cam) {
    if(apply(settings.exposureUs,settings.analogGain,settings.digitalGain,settings.ispGain,settings.hcg)!=0)
      throw std::runtime_error("ISP fixed exposure/gain apply failed");
    cam.skip_frames(20);
    reference=readback();
  }
  std::array<uint32_t,12> verify() const {
    auto now=readback();FixedCameraSettings::validateHold(reference,now);return now;
  }
};
