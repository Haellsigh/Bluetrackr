#pragma once

#include <tuple>

namespace blt::algorithms {

class madgwick {
 public:
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

  float beta_ = 0.1f;
};

}  // namespace blt::algorithms