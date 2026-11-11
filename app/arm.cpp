//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include "main.h"
#include "motor_monitor.h"
#include <cmath>
#include <algorithm>

#define M_PI 3.14159265359

constexpr float DTR = M_PI / 180;
constexpr float RTD = 180 / M_PI;

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward)
{
}

bool Arm::feedbackReady() //ok
{
    if (!joint0_.connect_.check() || !joint1_.connect_.check()) { return false; }
    if (joint0_.mode_ == Motor::Mode::INIT || joint1_.mode_ == Motor::Mode::INIT) { return false; }
    return true;
}

bool Arm::withinLimits(const float q[2]) const //ok
{
    if (q[0] <= limits_.q_min[0] || q[0] >= limits_.q_max[0])return false;
    if (q[1] <= limits_.q_min[1] || q[1] >= limits_.q_max[1])return false;
    return true;
}

bool Arm::calibrate(float q0, float q1) //rad
{
    if (!(mode_ == Mode::Disabled && feedbackReady())) { return false; }
    float q[2] = {q0, q1};
    if (!withinLimits(q)) { return false; }

    //电机重置
    joint0_.reset(), joint1_.reset();
    joint0_.resetFeedbackAngle(q0 * RTD), joint1_.resetFeedbackAngle(q1 * RTD);

    //位置维持
    updateFeedback();
    reference_ = Reference{};
    reference_.q[0] = state_.q[0];
    reference_.q[1] = state_.q[1];
    calibrated_ = true;
    return true;
}

bool Arm::enable()
{
    if (mode_ == Mode::Joint)return true;
    //reference设为当前位置
    Reference reference0{};
    reference0.q[0] = state_.q[0], reference0.q[1] = state_.q[1];
    setReference(reference0);
    //电机初始化
    joint0_.ppid_.reset(), joint0_.spid_.reset();
    joint1_.ppid_.reset(), joint1_.spid_.reset();
    allMotorsOn();
    mode_ = Mode::Joint;
    return true;
}

void Arm::disable()
{
    //todo
    if (mode_ == Mode::Disabled)return;
    trajAbort();
    allMotorsShutOff();

    mode_ = Mode::Disabled;
}

bool Arm::setReference(const Reference& reference)
{
    if (!withinLimits(reference.q)) { return false; }
    // if (trajRunning()) { trajAbort(); }
    reference_ = reference;
    return true;
}

bool Arm::addJointReference(float speed0, float speed1, float dt_s)
{
    const float speedFactor = 2.0f;
    if (mode_ != Mode::Joint || trajRunning()) {
        return false;
    }
    float speed[2] = {speed0/speedFactor,speed1/speedFactor};
    CartesianReference ncpos = ArmKinematics::forward(reference_.q);
    //写入积分位置
    ncpos.x += speed[0]*dt_s,ncpos.y += speed[1]*dt_s;

    Reference npos = reference_;npos.dq[0] = 0,npos.dq[1] = 0;npos.ddq[0] = 0,npos.ddq[1] = 0;
    int elbow = (reference_.q[1] >= 0.0f) ? 1 : -1;//防止跳肘
    if (!ArmKinematics::inverse(ncpos,reference_.q,elbow,npos.q))return false;//逆运动
    if (!withinLimits(npos.q))return false;//超限
    return setReference(npos);
}

bool Arm::trajectoryReady()
{
    //ok
    return !trajRunning() && mode_ == Mode::Joint && calibrated_ && feedbackReady();
}

bool Arm::trajStart(float x_m, float y_m, float duration_s)
{
  if(duration_s <= 0.0f) {abort_reason_ = -6;return false;}
  if(!trajectoryReady()) {abort_reason_ = -5;return false;}

    CartesianReference spos, tpos;
    tpos.x = x_m, tpos.y = y_m;
    spos= ArmKinematics::forward(reference_.q);
    //检查是否可行
    if (!ArmKinematics::lineReachable(spos, tpos))
    {
      abort_reason_ = -4;
      return false;
    }
    if (!trajectory_.start(spos, tpos, duration_s)){abort_reason_ = -3;return false;}
    //设置状态参数
    trajectory_start_tick_ = HAL_GetTick();
    trajectory_type_ = TrajectoryType::Cartesian;
    joint_trajectory_duration_ = duration_s;
    trajectory_finished_ = false;

    return true;
}

bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s)
{
    trajectory_start_tick_ = HAL_GetTick();
    trajectory_type_ = TrajectoryType::Joint;
    trajectory_finished_ = false;

    return false;
}

void Arm::trajAbort()
{
    //ok
    trajectory_.abort();
    trajectory_type_ = TrajectoryType::None;
    joint_trajectory_duration_ = 0;
    reference_.dq[0] = reference_.dq[1] = 0.0f;
    reference_.ddq[0] = reference_.ddq[1] = 0.0f;
    trajectory_finished_ = false;
}


void Arm::trajectoryPlanner()
{
    //轨道监测
    float elapsed_s = static_cast<float>(HAL_GetTick() - trajectory_start_tick_) * 0.001f;
    if (elapsed_s>=joint_trajectory_duration_)
    {//结束了

        trajAbort();
        trajectory_finished_ = true;
    }
    if (trajectory_finished_)
    {
        //轨迹完成
        float next_q[2]={0,0};
        if(!trajGetQueue(next_q)) {
          //轨迹走完
          abort_reason_ = 1;
          return;
        };

        if(!trajStart(next_q[0],next_q[1],traj_std_duration_s_)){
          //发生异常
            trajAbort();
          abort_reason_ += 20;
        }
        return;
    }
    //如果还在跑，取样
    const CartesianReference point = trajectory_.sample(elapsed_s);
    Reference nextReference;
    bool rep1 = ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, nextReference.q);
    bool rep2 = ArmKinematics::derivatives(point, reference_.q, nextReference.dq, nextReference.ddq);
    bool rep3 = setReference(nextReference);
    if (!(rep1&&rep2&&rep3)){trajAbort();}
}

void Arm::updateFeedback()
{
    // 更新参数
    state_.q[0] = joint0_.realAngle() * DTR, state_.q[1] = joint1_.realAngle() * DTR;
    state_.dq[0] = joint0_.realSpeed() * DTR, state_.dq[1] = joint1_.realSpeed() * DTR;
}

void Arm::handle()
{
    updateFeedback();
    if (mode_ == Mode::Disabled)return;
    if (trajectory_.running())
    {
        trajectoryPlanner();
    }
    writeMotorCommands();
}

void Arm::writeMotorCommands()
{
    joint0_.method_ = Motor::POSITION_SPEED;
    joint1_.method_ = Motor::POSITION_SPEED;
    joint0_.setAngleSpeed(
        reference_.q[0] * RTD, reference_.dq[0] * RTD,feedforward_.k_acc[0] * reference_.ddq[0]);
    joint1_.setAngleSpeed(
        reference_.q[1] * RTD, reference_.dq[1] * RTD, feedforward_.k_acc[1] * reference_.ddq[1]);
}
