#pragma once

#include <cstdint>

#include <blt/memory/manip.hh>

namespace blt::lsm9ds1 {

using namespace blt::memory;

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

struct Control1Gyro : reg<0x10> {
  using Bandwidth = reg::field<0, 2>;
  using FullScale = reg::field<3, 2>;
  using DataRate  = reg::field<5, 3>;
};

struct Control2Gyro : reg<0x11> {
  using OutputSelection = reg::field<0, 2>;
  using IntSelection    = reg::field<2, 2>;
};

struct Control3Gyro : reg<0x12> {
  using HighPassCutoffFreq = reg::field<0, 4>;
  using HighPassEnable     = reg::field<6>;
  using LowPowerEnable     = reg::field<7>;
};

struct GyroOrientation : reg<0x13> {
  using UserOrientation = reg::field<0, 3>;
  using ZSign           = reg::field<3>;
  using YSign           = reg::field<4>;
  using XSign           = reg::field<5>;
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
  using High = reg::field<8, 4>;
};

struct Status1 : reg<0x17> {
  using XLDA        = reg::field<0>;
  using GDA         = reg::field<1>;
  using TDA         = reg::field<2>;
  using BOOT_STATUS = reg::field<3>;
  using INACT       = reg::field<4>;
  using IG_G        = reg::field<5>;
  using IG_XL       = reg::field<6>;
};

struct GyroX : reg<0x18, 16> {
  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 8>;
};
struct GyroY : reg<0x1A, 16> {
  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 8>;
};
struct GyroZ : reg<0x1C, 16> {
  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 8>;
};

struct GyroControl4 : reg<0x1E> {
  using Int4D         = reg::field<0>;
  using LatchedInt    = reg::field<1>;
  using XOutputEnable = reg::field<3>;
  using XOutputEnable = reg::field<4>;
  using XOutputEnable = reg::field<3>;
};

struct Control5Acc : reg<0x1F> {
  using XOutputEnable = reg::field<3>;
  using YOutputEnable = reg::field<4>;
  using ZOutputEnable = reg::field<5>;
  using Decimation    = reg::field<6, 2>;
};

struct Control6Acc : reg<0x20> {
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

struct Control7Acc : reg<0x21> {
  using HPIS1 = reg::field<0>;
  struct FDS : reg::field<2> {
    static constexpr field::value_t kBypassInternalFilter = 0;
    static constexpr field::value_t kUseInternalFilter    = 1;
  };
  struct LPFilterCutoff : reg::field<5, 2> {
    static constexpr field::value_t kOdr_50  = 0b00;
    static constexpr field::value_t kOdr_100 = 0b01;
    static constexpr field::value_t kOdr_9   = 0b10;
    static constexpr field::value_t kOdr_400 = 0b11;
  };
  using HR = reg::field<7>;
};

struct Control8 : reg<0x22> {
  struct SWReset : reg::field<0> {
    static constexpr field::value_t kResetDevice;
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

struct Control9 : reg<0x23> {
  using StopOnFifoThreshold   = reg::field<0>;
  using FifoEnable            = reg::field<1>;
  using I2CDisable            = reg::field<2>;
  using DataAvailableEnable   = reg::field<3>;
  using FifoTemperatureEnable = reg::field<4>;
  using GyroSleepEnable       = reg::field<6>;
};

struct Control10 : reg<0x24> {
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

struct Status2 : reg<0x27> {
  using XLDA        = reg::field<0>;
  using GDA         = reg::field<1>;
  using TDA         = reg::field<2>;
  using BOOT_STATUS = reg::field<3>;
  using INACT       = reg::field<4>;
  using IG_G        = reg::field<5>;
  using IG_XL       = reg::field<6>;
};

struct AccXLow : reg<0x28, 16> {
  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 8>;
};

struct AccYLow : reg<0x2A, 16> {
  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 8>;
};

struct AccZLow : reg<0x2C, 16> {
  using Low  = reg::field<0, 8>;
  using High = reg::field<8, 8>;
};

struct FifoControl : reg<0x2E> {
  using FifoThresholdLevel = reg::field<0, 5>;
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

}  // namespace blt::lsm9ds1