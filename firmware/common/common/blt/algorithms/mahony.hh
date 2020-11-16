#pragma once

#include <tuple>

namespace blt::algorithms {

class mahony {
 public:
  mahony(float kp = 0.5f, float ki = 0.f) : twoKp_(2.f * kp), twoKi_(2.f * ki) {}
  void update(float dt,
              float gx,
              float gy,
              float gz,
              float ax,
              float ay,
              float az,
              float mx,
              float my,
              float mz);
  void update(float dt, float gx, float gy, float gz, float ax, float ay, float az);

  std::tuple<float, float, float> get();

 private:
  bool  angles_up_to_date_ = false;
  float roll_, pitch_, yaw_;

  float q0_ = 1.f, q1_ = 0.f, q2_ = 0.f, q3_ = 0.f;  // Quaternion of sensor

  float twoKp_       = 2.f * 0.5f;                                   // 2 * proportional gain (Kp)
  float twoKi_       = 2.f * 0.0f;                                   // 2 * integral gain (Ki)
  float integralFBx_ = 0.f, integralFBy_ = 0.f, integralFBz_ = 0.f;  // Integral error terms scaled
};

}  // namespace blt::algorithms