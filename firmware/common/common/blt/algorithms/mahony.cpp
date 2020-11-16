#include <blt/algorithms/fastmath.hh>
#include <blt/algorithms/mahony.hh>

#include <cmath>

namespace blt::algorithms {

using namespace fastmath;

/**
 * \brief Updates filter from gyroscope, accelerometer and magnetometer data.
 *
 * \note Accelerometer and magnetometer data are normalized before usage.
 *
 * \note Gyroscope data is in degrees/s.
 *
 * \param dt Time delta in seconds since the last update.
 * \param gx Gyroscope x axis angular velocity in degrees/s.
 * \param gy Gyroscope y axis angular velocity in degrees/s.
 * \param gz Gyroscope z axis angular velocity in degrees/s.
 * \param ax Accelerometer x axis linear acceleration in any unit.
 * \param ay Accelerometer y axis linear acceleration in any unit.
 * \param az Accelerometer z axis linear acceleration in any unit.
 * \param mx Magnetometer x axis magnetic field in any unit.
 * \param my Magnetometer y axis magnetic field in any unit.
 * \param mz Magnetometer z axis magnetic field in any unit.
 */
void mahony::update(float dt,
                    float gx,
                    float gy,
                    float gz,
                    float ax,
                    float ay,
                    float az,
                    float mx,
                    float my,
                    float mz)
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Use IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  if ((mx == 0.f) && (my == 0.f) && (mz == 0.f)) {
    return update(dt, gx, gy, gz, ax, ay, az);
  }

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.f) && (ay == 0.f) && (az == 0.f))) {
    // Normalise accelerometer measurement
    recipNorm = inverse_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = inverse_sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0_ * q0_;
    q0q1 = q0_ * q1_;
    q0q2 = q0_ * q2_;
    q0q3 = q0_ * q3_;
    q1q1 = q1_ * q1_;
    q1q2 = q1_ * q2_;
    q1q3 = q1_ * q3_;
    q2q2 = q2_ * q2_;
    q2q3 = q2_ * q3_;
    q3q3 = q3_ * q3_;

    // Reference direction of Earth's magnetic field
    hx = 2.f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = std::sqrt(hx * hx + hy * hy);
    bz = 2.f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi_ > 0.f) {
      // integral error scaled by Ki
      integralFBx_ += twoKi_ * halfex * dt;
      integralFBy_ += twoKi_ * halfey * dt;
      integralFBz_ += twoKi_ * halfez * dt;
      gx += integralFBx_;  // apply integral feedback
      gy += integralFBy_;
      gz += integralFBz_;
    } else {
      integralFBx_ = 0.f;  // prevent integral windup
      integralFBy_ = 0.f;
      integralFBz_ = 0.f;
    }

    // Apply proportional feedback
    gx += twoKp_ * halfex;
    gy += twoKp_ * halfey;
    gz += twoKp_ * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);  // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0_;
  qb = q1_;
  qc = q2_;
  q0_ += (-qb * gx - qc * gy - q3_ * gz);
  q1_ += (qa * gx + qc * gz - q3_ * gy);
  q2_ += (qa * gy - qb * gz + q3_ * gx);
  q3_ += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = inverse_sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;
  angles_up_to_date_ = false;
}

/**
 * \brief Updates filter from gyroscope and accelerometer data.
 *
 * \note Accelerometer data is normalized before usage.
 *
 * \note Gyroscope data is in degrees/s.
 *
 * \param dt Time delta in seconds since the last update.
 * \param gx Gyroscope x axis angular velocity in degrees/s.
 * \param gy Gyroscope y axis angular velocity in degrees/s.
 * \param gz Gyroscope z axis angular velocity in degrees/s.
 * \param ax Accelerometer x axis linear acceleration in any unit.
 * \param ay Accelerometer y axis linear acceleration in any unit.
 * \param az Accelerometer z axis linear acceleration in any unit.
 */
void mahony::update(float dt, float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.f) && (ay == 0.f) && (az == 0.f))) {
    // Normalise accelerometer measurement
    recipNorm = inverse_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = q1_ * q3_ - q0_ * q2_;
    halfvy = q0_ * q1_ + q2_ * q3_;
    halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

    // Error is sum of cross product between estimated
    // and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi_ > 0.f) {
      // integral error scaled by Ki
      integralFBx_ += twoKi_ * halfex * dt;
      integralFBy_ += twoKi_ * halfey * dt;
      integralFBz_ += twoKi_ * halfez * dt;
      gx += integralFBx_;  // apply integral feedback
      gy += integralFBy_;
      gz += integralFBz_;
    } else {
      integralFBx_ = 0.f;  // prevent integral windup
      integralFBy_ = 0.f;
      integralFBz_ = 0.f;
    }

    // Apply proportional feedback
    gx += twoKp_ * halfex;
    gy += twoKp_ * halfey;
    gz += twoKp_ * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);  // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0_;
  qb = q1_;
  qc = q2_;
  q0_ += (-qb * gx - qc * gy - q3_ * gz);
  q1_ += (qa * gx + qc * gz - q3_ * gy);
  q2_ += (qa * gy - qb * gz + q3_ * gx);
  q3_ += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = inverse_sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;
  angles_up_to_date_ = false;
}

/**
 * \brief Computes and return the roll, pitch and yaw angles in degrees.
 *
 * \note pitch is in range [-90°; +90°]
 *
 * \note roll and yaw are in range [-180°; 180°]
 *
 * \return A tuple containing the roll, pitch and yaw angles in this order.
 */
std::tuple<float, float, float> mahony::get()
{
  if (angles_up_to_date_)
    return {roll_, pitch_, yaw_};

  constexpr float pi        = 3.141592653589793f;
  constexpr float rad_2_deg = 180.f / pi;

  roll_              = rad_2_deg * std::atan2(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_);
  pitch_             = rad_2_deg * std::asin(-2.f * (q1_ * q3_ - q0_ * q2_));
  yaw_               = rad_2_deg * std::atan2(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_);
  angles_up_to_date_ = true;

  return {roll_, pitch_, yaw_};
}

}  // namespace blt::algorithms
