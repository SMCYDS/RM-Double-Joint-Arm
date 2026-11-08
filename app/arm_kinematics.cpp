#include "arm_kinematics.h"
#include <cmath>
#include <algorithm>

#define M_PI 3.14159265359

//motion verification passed
bool regularRadius(float radius_squared) {
  const float c = (radius_squared - ArmKinematics::l0 * ArmKinematics::l0 -
          ArmKinematics::l1 * ArmKinematics::l1) /
         (2.f * ArmKinematics::l0 * ArmKinematics::l1);// 余弦定理解肘关节角度
  return 1.f - c * c >= ArmKinematics::min_sin_q1 * ArmKinematics::min_sin_q1;
}
float unwrap(float angle, float previous) {
  //依据上一角度解算到合适的角度
  return angle + M_PI*2 * std::round((previous - angle) / M_PI/2);
}


//正运动学
CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference p2;
  p2.x=l0*cosf(q[0])+l1*cosf(q[0]+q[1]), p2.y=l0*sinf(q[0])+l1*sinf(q[0]+q[1]);
  return p2;
}

//逆运动学
float err_q[4]={-0.999f,-0,0,0};
bool ArmKinematics::inverse(const CartesianReference& p,
                      const float previous_q[2], int elbow, float q[2]) {

  float pl2 = p.x*p.x+p.y*p.y;
  if (!(fabs(elbow)==1)||!regularRadius(pl2)) {
    if(err_q[0] == -0.999f){err_q[0] = p.x, err_q[1] = p.y, err_q[2] = previous_q[0],err_q[3] = previous_q[1];}
    return false;//右肘是1
  }

  float cos_q1 = -(l0*l0+l1*l1-pl2)/(2*l1*l0); // 余弦定理
  q[0] = unwrap(atan2(p.y,p.x)-elbow*acosf((pl2+l0*l0-l1*l1)/(2*sqrt(pl2)*l0)),previous_q[0]);
  q[1] = unwrap(elbow*acosf(cos_q1),previous_q[1]);
  return true;
}

//线可过     done
bool ArmKinematics::lineReachable(const CartesianReference& s,
                                const CartesianReference& e) {
  float dx = e.x-s.x,dy = e.y-s.y,len2 = dx*dx+dy*dy,elen2 = e.x*e.x+e.y*e.y,slen2 = s.x*s.x+s.y*s.y;
  if (sqrtf(elen2)<fabs(l0-l1)||sqrtf(elen2)>fabs(l0+l1))return false; //位置超限
  float t = std::min(std::max(-(s.x * dx + s.y * dy) / len2, 0.0f),1.0f);
  float cx = s.x + t * dx,cy = s.y + t * dy;
  // if (!powf(s.x + t * dx,2) + powf(s.y + t * dy,2) >= powf(std::fabs(l1 - l0),2))return false;//距离过死区
  return regularRadius(cx*cx+cy*cy) && regularRadius(elen2) && regularRadius(slen2);
}

//微分   done
bool ArmKinematics::derivatives(const CartesianReference& point, const float q[2], float dq[2], float ddq[2]) {
  //点的运动解析到关节角度
  //point和q都是输入??有const
  float q0 = q[0],q1 = q[1],x=point.x,y=point.y,vx=point.dx,vy=point.dy;

  const float j00 = -l0 * sinf(q0) - l1*sinf(q0+q1);
  const float j01 = -l1 * sinf(q0+q1);
  const float j10 = l0 * cosf(q0) + l1 * cosf(q0+q1);
  const float j11 = l1 * cosf(q0+q1);
  float det = l0 * l1 * sinf(q1); //求逆所以要除以这个
  dq[0] = (j11 * vx - j01 * vy) / det;
  dq[1] = (-j10 * vx + j00 * vy) / det;

  float dq01 = dq[0]+dq[1]; //小臂合角速度
  const float jdot00 =-l0 * cosf(q0) * dq[0]-l1 * cosf(q0 + q1) * dq01;
  const float jdot01 =-l1 * cosf(q0 + q1) * dq01;
  const float jdot10 =-l0 * sinf(q0) * dq[0]-l1 * sinf(q0 + q1) * dq01;
  const float jdot11 =-l1 * sinf(q0 + q1) * dq01;

  // a = ddp - J_dot * dq
  const float ax = point.ddx - (jdot00 * dq[0] + jdot01 * dq[1]);
  const float ay = point.ddy - (jdot10 * dq[0] + jdot11 * dq[1]);

  // ddq = J^(-1) * a
  ddq[0] = ( j11 * ax - j01 * ay) / det;
  ddq[1] = (-j10 * ax + j00 * ay) / det;

  return true;
}
