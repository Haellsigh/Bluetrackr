#pragma once

#include <cstdint>

#include <blt/memory/manip.hh>

namespace blt::lsm9ds1 {

using namespace blt::memory2;

struct ActivityThreshold : reg<0x04> {
  using InactivityThreshold     = reg::field<0, 7>;
  using SleepOnInactivityEnable = reg::field<7>;
};

struct InactivityDuration : reg<0x05> {};

struct IntConfigAcc : reg<0x06> {
  using EnableOnXLowEvent   = reg::field<0>;
  using EnableOnXHighEvent  = reg::field<1>;
  using EnableOnYLowEvent   = reg::field<2>;
  using EnableOnYHighEvent  = reg::field<3>;
  using EnableOnZLowEvent   = reg::field<4>;
  using EnableOnZHighEvent  = reg::field<5>;
  using Enable6DirDetection = reg::field<6>;
  using LogicAndOr          = reg::field<7>;
};

struct IntXThresholdAcc : reg<0x07> {};
struct IntYThresholdAcc : reg<0x08> {};
struct IntZThresholdAcc : reg<0x09> {};

struct IntDurationAcc : reg<0x0A> {
  using Duration   = reg::field<0, 7>;
  using WaitEnable = reg::field<7>;
};

struct GyroRateRefHighPass : reg<0x0B> {};

struct Int1AGControl : reg<0x0C> {
  using AccDataReady  = reg::field<0>;
  using GyroDataReady = reg::field<1>;
  using BootStatus    = reg::field<2>;
  using FifoThreshold = reg::field<3>;
  using Overrun       = reg::field<4>;
  using FSS5          = reg::field<5>;
  using AccInt        = reg::field<6>;
  using GyroInt       = reg::field<7>;
};

struct Int2AGControl : reg<0x0D> {
  using AccDataReady  = reg::field<0>;
  using GyroDataReady = reg::field<1>;
  using TempDataReady = reg::field<2>;
  using FifoThreshold = reg::field<3>;
  using Overrun       = reg::field<4>;
  using FSS5          = reg::field<5>;
  using Inactivity    = reg::field<7>;
};

struct WhoAmI : reg<0x0F> {};

struct Control1AG : reg<0x10> {
  struct Bandwidth : reg::field<0, 2> {
    static constexpr field::value_t kSmall  = 0b00;
    static constexpr field::value_t kTiny   = 0b01;
    static constexpr field::value_t kMedium = 0b10;
    static constexpr field::value_t kLarge  = 0b11;
  };
  struct FullScale : reg::field<3, 2> {
    static constexpr field::value_t k245dps  = 0b00;
    static constexpr field::value_t k500dps  = 0b01;
    static constexpr field::value_t k2000dps = 0b11;
  };
  struct DataRate : reg::field<5, 3> {
    static constexpr field::value_t kPowerDown = 0b000;
    static constexpr field::value_t k14_9Hz    = 0b001;
    static constexpr field::value_t k59_5Hz    = 0b010;
    static constexpr field::value_t k119Hz     = 0b011;
    static constexpr field::value_t k238Hz     = 0b100;
    static constexpr field::value_t k476Hz     = 0b101;
    static constexpr field::value_t k952Hz     = 0b110;
  };
};

struct Control2AG : reg<0x11> {
  using OutputSelection = reg::field<0, 2>;
  using IntSelection    = reg::field<2, 2>;
};

struct Control3AG : reg<0x12> {
  using HighPassCutoffFreq = reg::field<0, 4>;  //!< Between 0 and 9
  using HighPassEnable     = reg::field<6>;     //!< 1: enable
  using LowPowerEnable     = reg::field<7>;     //!< 1: enable
};

struct GyroOrientation : reg<0x13> {
  struct UserOrientation : reg::field<0, 3> {
    static constexpr field::value_t kXYZ = 0b000;
    static constexpr field::value_t kXZY = 0b001;
    static constexpr field::value_t kYXZ = 0b010;
    static constexpr field::value_t kYZX = 0b011;
    static constexpr field::value_t kZXY = 0b100;
    static constexpr field::value_t kZYX = 0b101;
  };
  using FlipZ = reg::field<3>;
  using FlipY = reg::field<4>;
  using FlipX = reg::field<5>;
};

struct GyroIntSource : reg<0x14> {
  using XLow   = reg::field<0>;
  using XHigh  = reg::field<1>;
  using YLow   = reg::field<2>;
  using YHigh  = reg::field<3>;
  using ZLow   = reg::field<4>;
  using ZHigh  = reg::field<5>;
  using Active = reg::field<6>;
};

struct Temperature : reg<0x15, 16> {
  using Value = reg::field<0, 12>;

  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 4, access_policy::ro, int16_t>;
};

struct Status1AG : reg<0x17> {
  using DataAvailableAcc  = reg::field<0>;
  using DataAvailableGyro = reg::field<1>;
  using DataAvailableTemp = reg::field<2>;
  using BOOT_STATUS       = reg::field<3>;
  using INACT             = reg::field<4>;
  using IG_G              = reg::field<5>;
  using IG_XL             = reg::field<6>;
};

struct Gyro : reg<0x18, 48> {
  using X = reg::field<0, 16, access_policy::ro, int16_t>;
  using Y = reg::field<16, 16, access_policy::ro, int16_t>;
  using Z = reg::field<32, 16, access_policy::ro, int16_t>;
};

struct Control4AG : reg<0x1E> {
  struct Int4D : reg::field<0> {
    static constexpr field::value_t kUse6D = 0;
    static constexpr field::value_t kUse4D = 0;
  };
  using LatchedInt    = reg::field<1>;
  using XOutputEnable = reg::field<3>;
  using YOutputEnable = reg::field<4>;
  using ZOutputEnable = reg::field<5>;
};

struct Control5AG : reg<0x1F> {
  using XOutputEnable = reg::field<3>;
  using YOutputEnable = reg::field<4>;
  using ZOutputEnable = reg::field<5>;
  struct Decimation : reg::field<6, 2> {
    static constexpr field::value_t kNone     = 0b00;
    static constexpr field::value_t k2Samples = 0b01;
    static constexpr field::value_t k4Samples = 0b10;
    static constexpr field::value_t k8Samples = 0b11;
  };
};

struct Control6AG : reg<0x20> {
  struct FilterBandwidth : reg::field<0, 2> {
    static constexpr field::value_t k408Hz = 0b00;
    static constexpr field::value_t k211Hz = 0b01;
    static constexpr field::value_t k105Hz = 0b10;
    static constexpr field::value_t k50Hz  = 0b11;
  };
  struct Bandwidth : reg::field<2> {
    static constexpr field::value_t kFromODR      = 0;
    static constexpr field::value_t kFromFilterBW = 1;
  };
  struct Scale : reg::field<3, 2> {
    static constexpr field::value_t k2g  = 0b00;
    static constexpr field::value_t k16g = 0b01;
    static constexpr field::value_t k4g  = 0b10;
    static constexpr field::value_t k8g  = 0b11;
  };
  struct OutputDataRate : reg::field<5, 3> {
    static constexpr field::value_t kPowerDown = 0b000;
    static constexpr field::value_t k10Hz      = 0b001;
    static constexpr field::value_t k50Hz      = 0b010;
    static constexpr field::value_t k119Hz     = 0b011;
    static constexpr field::value_t k238Hz     = 0b100;
    static constexpr field::value_t k476Hz     = 0b101;
    static constexpr field::value_t k952Hz     = 0b110;
  };
};

struct Control7AG : reg<0x21> {
  using HPIS1         = reg::field<0>;
  using DisableFilter = reg::field<2>;
  struct FilterCutoff : reg::field<5, 2> {
    static constexpr field::value_t kOdr_50  = 0b00;
    static constexpr field::value_t kOdr_100 = 0b01;
    static constexpr field::value_t kOdr_9   = 0b10;
    static constexpr field::value_t kOdr_400 = 0b11;
  };
  using EnableHighResolution = reg::field<7>;
};

struct Control8AG : reg<0x22> {
  struct SWReset : reg::field<0> {
    static constexpr field::value_t kResetDevice = 1;
  };
  struct Endianness : reg::field<1> {
    static constexpr field::value_t kLSBFirst = 0;
    static constexpr field::value_t kMSBFirst = 1;
  };
  using IF_ADD_INC         = reg::field<2>;
  using SPIModeSelection   = reg::field<3>;
  using PushPull_OpenDrain = reg::field<4>;
  using H_LACTIVE          = reg::field<5>;
  using BlockDataUpdate    = reg::field<6>;
  using Boot               = reg::field<7>;
};

struct Control9AG : reg<0x23> {
  using StopOnFifoThreshold   = reg::field<0>;
  using FifoEnable            = reg::field<1>;
  using I2CDisable            = reg::field<2>;
  using DataAvailableEnable   = reg::field<3>;
  using FifoTemperatureEnable = reg::field<4>;
  using GyroSleepEnable       = reg::field<6>;
};

struct Control10AG : reg<0x24> {
  using AccSelfTestEnable  = reg::field<0>;
  using GyroSelfTestEnable = reg::field<2>;
};

struct AccIntSource : reg<0x26> {
  using XLow   = reg::field<0>;
  using XHigh  = reg::field<1>;
  using YLow   = reg::field<2>;
  using YHigh  = reg::field<3>;
  using ZLow   = reg::field<4>;
  using ZHigh  = reg::field<5>;
  using Active = reg::field<6>;
};

struct Status2AG : reg<0x27> {
  using XLDA        = reg::field<0>;
  using GDA         = reg::field<1>;
  using TDA         = reg::field<2>;
  using BOOT_STATUS = reg::field<3>;
  using INACT       = reg::field<4>;
  using IG_G        = reg::field<5>;
  using IG_XL       = reg::field<6>;
};

struct Acc : reg<0x28, 48> {
  using X = reg::field<0, 16, access_policy::ro, int16_t>;
  using Y = reg::field<16, 16, access_policy::ro, int16_t>;
  using Z = reg::field<32, 16, access_policy::ro, int16_t>;
};

struct FifoControl : reg<0x2E> {
  using FifoThresholdLevel = reg::field<0, 5>;  //!<  < 31
  struct FifoModeSelection : reg::field<5, 3> {
    static constexpr field::value_t kOff                    = 0b000;
    static constexpr field::value_t kFifo                   = 0b001;
    static constexpr field::value_t kContinuousUntilTrigger = 0b011;
    static constexpr field::value_t kBypass                 = 0b100;
    static constexpr field::value_t kContinuous             = 0b110;
  };
};

struct FifoStatusControl : reg<0x2F> {
  using SampleCount = reg::field<0, 6>;
  using Overrun     = reg::field<6>;
  using Threshold   = reg::field<7>;
};

struct IntConfigGyro : reg<0x30> {
  using EnableOnXLowEvent  = reg::field<0>;
  using EnableOnXHighEvent = reg::field<1>;
  using EnableOnYLowEvent  = reg::field<2>;
  using EnableOnYHighEvent = reg::field<3>;
  using EnableOnZLowEvent  = reg::field<4>;
  using EnableOnZHighEvent = reg::field<5>;
  using Latch              = reg::field<6>;
  using LogicAndOr         = reg::field<7>;
};

struct GyroIntXThreshold : reg<0x31, 16> {
  using Threshold        = reg::field<0, 15>;
  using ThresholdLow     = reg::field<0, 8>;
  using ThresholdHigh    = reg::field<8, 7>;
  using DecrementOrReset = reg::field<15>;
};

struct GyroIntYThreshold : reg<0x33, 16> {
  using Threshold        = reg::field<0, 15>;
  using ThresholdLow     = reg::field<0, 8>;
  using ThresholdHigh    = reg::field<8, 7>;
  using DecrementOrReset = reg::field<15>;
};

struct GyroIntZThreshold : reg<0x35, 16> {
  using Threshold        = reg::field<0, 15>;
  using ThresholdLow     = reg::field<0, 8>;
  using ThresholdHigh    = reg::field<8, 7>;
  using DecrementOrReset = reg::field<15>;
};

struct GyroIntDuration : reg<0x37> {
  using Duration = reg::field<0, 7>;
  using Wait     = reg::field<7>;
};

/*!
 * Magnetometer registers
 */
struct MagXOffset : reg<0x05, 16> {
  using Offset = reg::field<0, 16, access_policy::rw, int16_t>;
};

struct MagYOffset : reg<0x07, 16> {
  using Offset = reg::field<0, 16, access_policy::rw, int16_t>;
};

struct MagZOffset : reg<0x09, 16> {
  using Offset = reg::field<0, 16, access_policy::rw, int16_t>;
};

// struct WhoAmi {}; // same as acc/gyro

struct Ctrl1M : reg<0x20> {
  using EnableSelfTest = reg::field<0>;
  using EnableFastODR  = reg::field<1>;
  struct DataRate : reg::field<2, 3> {
    static constexpr field::value_t k0_625Hz = 0b000;
    static constexpr field::value_t k1_25Hz  = 0b001;
    static constexpr field::value_t k2_5Hz   = 0b010;
    static constexpr field::value_t k5Hz     = 0b011;
    static constexpr field::value_t k10Hz    = 0b100;
    static constexpr field::value_t k20Hz    = 0b101;
    static constexpr field::value_t k40Hz    = 0b110;
    static constexpr field::value_t k80Hz    = 0b111;
  };
  struct XYOperativeMode : reg::field<5, 2> {
    static constexpr field::value_t kLowPerf       = 0b00;
    static constexpr field::value_t kMediumPerf    = 0b01;
    static constexpr field::value_t kHighPerf      = 0b10;
    static constexpr field::value_t kUltraHighPerf = 0b11;
  };
  using EnableTemperatureComp = reg::field<7>;
};

struct Ctrl2M : reg<0x21> {
  using SoftReset    = reg::field<2>;
  using RebootMemory = reg::field<3>;
  struct ScaleSelection : reg::field<5, 2> {
    static constexpr field::value_t k4gauss  = 0b00;
    static constexpr field::value_t k8gauss  = 0b01;
    static constexpr field::value_t k12gauss = 0b10;
    static constexpr field::value_t k16gauss = 0b11;
  };
};

struct Ctrl3M : reg<0x22> {
  struct OperatingMode : reg::field<0, 2> {
    static constexpr field::value_t kContinuous       = 0b00;
    static constexpr field::value_t kSingleConversion = 0b01;
    static constexpr field::value_t kPowerDown        = 0b10;
    static constexpr field::value_t kPowerDown2       = 0b11;
  };
  struct SPIWrite : reg::field<2> {
    static constexpr field::value_t kWriteOnly = 0;
    static constexpr field::value_t kReadWrite = 1;
  };
  using LowPower   = reg::field<5>;
  using I2CDisable = reg::field<7>;
};

struct Ctrl4M : reg<0x23> {
  struct Endianness : reg::field<1> {
    static constexpr field::value_t kLSBFirst = 0;
    static constexpr field::value_t kMSBFirst = 1;
  };
  struct ZOperativeMode : reg::field<2, 2> {
    static constexpr field::value_t kLowPerf       = 0b00;
    static constexpr field::value_t kMediumPerf    = 0b01;
    static constexpr field::value_t kHighPerf      = 0b10;
    static constexpr field::value_t kUltraHighPerf = 0b11;
  };
};

struct Ctrl5M : reg<0x24> {
  using BlockDataUpdate = reg::field<6>;
  using FastRead        = reg::field<7>;
};

struct MStatus : reg<0x27> {
  using XDataReady     = reg::field<0>;
  using YDataReady     = reg::field<1>;
  using ZDataReady     = reg::field<2>;
  using XYZDataReady   = reg::field<3>;
  using XDataOverrun   = reg::field<4>;
  using YDataOverrun   = reg::field<5>;
  using ZDataOverrun   = reg::field<6>;
  using XYZDataOverrun = reg::field<7>;
};

struct Mag : reg<0x28, 48> {
  using X = reg::field<0, 16, access_policy::ro, int16_t>;
  using Y = reg::field<16, 16, access_policy::ro, int16_t>;
  using Z = reg::field<32, 16, access_policy::ro, int16_t>;
};

struct IntConfigM : reg<0x30> {
  using Enable  = reg::field<0>;
  using Latch   = reg::field<1>;
  using Active  = reg::field<2>;
  using EnableZ = reg::field<5>;
  using EnableY = reg::field<6>;
  using EnableX = reg::field<7>;
};

struct IntSourceM : reg<0x31> {
  using Int                      = reg::field<0>;
  using MeasurementRangeOverflow = reg::field<1>;
  using ZNegativeThreshold       = reg::field<2>;
  using YNegativeThreshold       = reg::field<3>;
  using XNegativeThreshold       = reg::field<4>;
  using ZPositiveThreshold       = reg::field<5>;
  using YPositiveThreshold       = reg::field<6>;
  using XPositiveThreshold       = reg::field<7>;
};

struct IntThresholdM : reg<0x32, 16> {
  using Threshold = reg::field<0, 15>;
};

}  // namespace blt::lsm9ds1