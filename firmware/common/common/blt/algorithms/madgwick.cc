#include <blt/algorithms/fastmath.hh>
#include <blt/algorithms/madgwick.hh>

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
void madgwick::update(float dt,
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
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2,
      _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer
  // normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    update(dt, gx, gy, gz, ax, ay, az);
    return;
  }

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
  // normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
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
    _2q0mx = 2.0f * q0_ * mx;
    _2q0my = 2.0f * q0_ * my;
    _2q0mz = 2.0f * q0_ * mz;
    _2q1mx = 2.0f * q1_ * mx;
    _2q0   = 2.0f * q0_;
    _2q1   = 2.0f * q1_;
    _2q2   = 2.0f * q2_;
    _2q3   = 2.0f * q3_;
    _2q0q2 = 2.0f * q0_ * q2_;
    _2q2q3 = 2.0f * q2_ * q3_;
    q0q0   = q0_ * q0_;
    q0q1   = q0_ * q1_;
    q0q2   = q0_ * q2_;
    q0q3   = q0_ * q3_;
    q1q1   = q1_ * q1_;
    q1q2   = q1_ * q2_;
    q1q3   = q1_ * q3_;
    q2q2   = q2_ * q2_;
    q2q3   = q2_ * q3_;
    q3q3   = q3_ * q3_;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3_ + _2q0mz * q2_ + mx * q1q1 + _2q1 * my * q2_ + _2q1 * mz * q3_ -
         mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3_ + my * q0q0 - _2q0mz * q1_ + _2q1mx * q2_ - my * q1q1 + my * q2q2 +
         _2q2 * mz * q3_ - my * q3q3;
    _2bx = std::sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2_ + _2q0my * q1_ + mz * q0q0 + _2q1mx * q3_ - mz * q1q1 + _2q2 * my * q3_ -
           mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3_ + _2bz * q1_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1_ * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2_ + _2bz * q0_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3_ - _4bz * q1_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2_ * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2_ - _2bz * q0_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1_ + _2bz * q3_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0_ - _4bz * q2_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3_ + _2bz * q1_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0_ + _2bz * q2_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

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
void madgwick::update(float dt, float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
  // normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = inverse_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0_;
    _2q1 = 2.0f * q1_;
    _2q2 = 2.0f * q2_;
    _2q3 = 2.0f * q3_;
    _4q0 = 4.0f * q0_;
    _4q1 = 4.0f * q1_;
    _4q2 = 4.0f * q2_;
    _8q1 = 8.0f * q1_;
    _8q2 = 8.0f * q2_;
    q0q0 = q0_ * q0_;
    q1q1 = q1_ * q1_;
    q2q2 = q2_ * q2_;
    q3q3 = q3_ * q3_;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 +
         _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 +
         _8q2 * q2q2 + _4q2 * az;
    s3        = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
    recipNorm = inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

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
std::tuple<float, float, float> madgwick::get()
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