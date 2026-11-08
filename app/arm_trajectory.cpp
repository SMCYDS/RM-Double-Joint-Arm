#include "arm_trajectory.h"

bool ArmTrajectory::start(const CartesianReference& start, const CartesianReference& end,
             float duration_s) {
  start_ = start,end_ = end,duration_s_ = duration_s;
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s){
  CartesianReference ans{};
  //对两个方向进行一维插值
  interpolate(start_.x,end_.x,elapsed_s,duration_s_,ans.x,ans.dx,ans.ddx);
  interpolate(start_.y,end_.y,elapsed_s,duration_s_,ans.y,ans.dy,ans.ddy);
  return ans;
}

void ArmTrajectory::interpolate(float start, float end, float elapsed_s,
                          float duration_s, float& position, float& velocity,
                          float& acceleration) {
  float x = elapsed_s/duration_s;//轨迹进行时间比例
  //根据五次轨迹曲线及其一二阶导数算得对应时间的x dx ddx
  float prate = 10*x*x*x-15*x*x*x*x+6*x*x*x*x*x,vrate = 30*x*x*x*x - 60*x*x*x + 30*x*x, arate = 120*x*x*x - 180*x*x + 60*x;
  position = prate*(end-start)+start;
  velocity = vrate*(end-start)/duration_s;
  acceleration = arate*(end-start)/(duration_s*duration_s);
}

void ArmTrajectory::abort() {
  running_ = false;
  start_ = CartesianReference{}, end_ = CartesianReference{}, duration_s_ = 0.0f;
}
