#include "LatestPreview.hpp"
#include <cassert>
#include <chrono>
#include <future>
#include <iostream>
#include <vector>
using namespace std::chrono_literals;
int main(){
  std::promise<void> entered, release, latest;
  auto ready=entered.get_future();auto go=release.get_future().share();auto done=latest.get_future();
  std::vector<int> received;
  LatestPreview<int> sender([&](int &n){
    received.push_back(n);
    if(n==1){entered.set_value();go.wait();}
    if(n==3)latest.set_value();
  });
  sender.submit(std::make_unique<int>(1));assert(ready.wait_for(3s)==std::future_status::ready);
  sender.submit(std::make_unique<int>(2));sender.submit(std::make_unique<int>(3));
  release.set_value();assert(done.wait_for(3s)==std::future_status::ready);sender.stop();
  assert((received==std::vector<int>{1,3}));assert(sender.completed()==2);
  std::promise<void> throwing;auto thrown=throwing.get_future();
  LatestPreview<int> broken([&](int&){throwing.set_value();throw std::runtime_error("sender failure");});
  broken.submit(std::make_unique<int>(4));assert(thrown.wait_for(3s)==std::future_status::ready);broken.stop();
  bool failed=false;try{broken.rethrowFailure();}catch(const std::runtime_error&){failed=true;}assert(failed);
  {LatestPreview<int> idle([](int&){});} // Destruction must wake an idle worker.
  std::cout<<"PASS latest preview: owned frames, stale replacement, sender failures, idle shutdown\n";
}
