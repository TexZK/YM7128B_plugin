#ifndef _PLUGIN_HPP_
#define _PLUGIN_HPP_

#include <mutex>
#include <type_traits>

#include "calfmoo/dsp-butterworth.hpp"
#include "calfmoo/dsp-resampler-zita.hpp"
#include "calfmoo/utils-ringbuffer.hpp"
#include "calfmoo/ym7128b.hpp"

#include "platform.hpp"

// ============================================================================

namespace plugin {

#pragma pack(push)
#pragma pack()

using namespace calfmoo::ym7128b;
using namespace calfmoo::dsp;

template <typename E>
constexpr auto to_underlying(E e) noexcept
{
    return static_cast<std::underlying_type_t<E>>(e);
}

enum class Param {
    kInputCutoff = YM7128B_Reg_Count,
    kOutputCutoff,
    kDry,
    kWet,
    kEngine,
    kCount,
};

typedef ZitaResampler InputResampler;
typedef ZitaResampler OutputResampler;

enum class Program {
    kOff,
    kDirect,
    kDuneArrakis,
    kDuneBaghdad,
    kDuneMorning,
    kDuneSequence,
    kDuneSietch,
    kDuneWarSong,
    kDuneWater,
    kDuneWormIntro,
    kDuneWormsuit,
    kCount,
};

typedef double Slider;

enum class DecibelRange {
    k50dB,
    k60dB,
    k70dB,
    k80dB,
    k90dB,
    k100dB,
    kCount
};

enum class Engine {
    kFixed = 0,
    kFloat,
    kIdeal,
    kCount,
};

// ----------------------------------------------------------------------------

extern char const* const kParamNames[to_underlying(Param::kCount)];

extern char const* const kProgramNames[to_underlying(Program::kCount)];
extern Register kProgramRegisters[to_underlying(Program::kCount)][YM7128B_Reg_Count];

extern char const* const kEngineNames[to_underlying(Engine::kCount)];

extern double const kDecibelSliderArgCoeff[to_underlying(DecibelRange::kCount)];
extern double const kDecibelSliderExpCoeff[to_underlying(DecibelRange::kCount)];
extern int const kDecibelSliderPowCoeff[to_underlying(DecibelRange::kCount)];

// ============================================================================

template <typename Output, typename Input> inline
Output const Clamp(Input const min, Input const max, Input const value)
{
    if (value < min) {
        return static_cast<Output>(min);
    }
    else if (max < value) {
        return static_cast<Output>(max);
    }
    else {
        return static_cast<Output>(value);
    }
}

// ----------------------------------------------------------------------------

template <typename Integral, Integral kIntegralMax> inline
Integral const SliderToIntegral(Slider const slider)
{
    Slider scale = static_cast<Slider>(kIntegralMax);
    Slider scaled = slider * scale + static_cast<Slider>(0.5);
    if (scaled >= scale) {
        return kIntegralMax;
    }
    else {
        return static_cast<Integral>(scaled);
    }
}

// ----------------------------------------------------------------------------

template <typename Integral, Integral kIntegralMax> inline
Slider const IntegralToSlider(Integral const value)
{
    Slider slider = static_cast<Slider>(value) / static_cast<Slider>(kIntegralMax);
    return slider;
}

// ----------------------------------------------------------------------------

inline double SliderToVolumeExp(Slider slider, DecibelRange range)
{
    if (slider) {
        auto index = to_underlying(range);
        double arg_coeff = kDecibelSliderArgCoeff[index];
        double exp_coeff = kDecibelSliderExpCoeff[index];
        double volume = exp(slider * arg_coeff) * exp_coeff;
        return volume;
    }
    else {
        return 0;
    }
}

// ----------------------------------------------------------------------------

inline Slider VolumeToSliderExp(double volume, DecibelRange range)
{
    if (volume) {
        auto index = to_underlying(range);
        double arg_coeff = 1.0 / kDecibelSliderArgCoeff[index];
        double exp_coeff = 1.0 / kDecibelSliderExpCoeff[index];
        Slider slider = log(volume * exp_coeff) * arg_coeff;
        return slider;
    }
    else {
        return 0;
    }
}

// ----------------------------------------------------------------------------

inline double SliderToVolumePow(Slider slider, DecibelRange range)
{
    auto index = to_underlying(range);
    int pow_coeff = kDecibelSliderPowCoeff[index];
    double volume = pow(slider, pow_coeff);
    return volume;
}

// ----------------------------------------------------------------------------

inline Slider VolumeToSliderPow(double volume, DecibelRange range)
{
    auto index = to_underlying(range);
    double pow_coeff = 1.0 / kDecibelSliderPowCoeff[index];
    Slider slider = pow(volume, pow_coeff);
    return slider;
}

// ----------------------------------------------------------------------------

template <typename T> inline
T const Lerp(T const value1, T const value2, T const amount)
{
    return (value2 - value1) * amount + value1;
}

// ----------------------------------------------------------------------------

template <typename T> inline
T const LerpInv(T const value1, T const value2, T const value)
{
    return (value - value1) / (value2 - value1);
}

// ----------------------------------------------------------------------------

inline double ToDecibels(double x)
{
    double db = log10(x) * 20.0;
    return db;
}

// ----------------------------------------------------------------------------

inline double FromDecibels(double db)
{
    double x = pow(10.0, db * 0.05);
    return x;
}

// ----------------------------------------------------------------------------

inline unsigned char FindFirstBitUL(unsigned long* index, unsigned long mask)
{
#ifdef _MSC_VER
    return _BitScanForward(index, mask);
#else
    for (unsigned long shift = 0; mask; ++shift, mask >>= 1) {
        if (mask & 1) {
            *index = shift;
            return 1;
        }
    }
    return 0;
#endif
}

// ----------------------------------------------------------------------------

template <typename Mask>
inline bool FindFirstBit(unsigned long& index, Mask mask)
{
    if (sizeof(Mask) > sizeof(unsigned long)) {
        unsigned long offset = 0;
        do {
            if (FindFirstBitUL(&index, static_cast<unsigned long>(mask))) {
                index += offset;
                return true;
            }
            mask >>= sizeof(unsigned long) * CHAR_BIT;
            offset += sizeof(unsigned long) * CHAR_BIT;
        } while (mask);
    }
    else {
        if (FindFirstBitUL(&index, static_cast<unsigned long>(mask))) {
            return true;
        }
    }
    return false;
}

// ============================================================================

class Plugin
{
public:
    enum {
        kInputChannelCount = 1,
        kOutputChannelCount = 2,

        kInputFilterOrder = 6,
        kOutputFilterOrder = 3,
    };

    typedef size_t Index;
    typedef float Sample;
    typedef float Parameter;
    typedef float Accumulator;
    typedef float SampleRate;

protected:
    typedef ChipBase<Accumulator, Accumulator, Index, Index> ChipBase;
    typedef ChipFixed<Accumulator, Accumulator, Index, Index> ChipFixed;
    typedef ChipFloat<Accumulator, Accumulator, Index, Index> ChipFloat;
    typedef ChipIdeal<Accumulator, Accumulator, Index, Index> ChipIdeal;

    typedef ButterworthParameters<kInputFilterOrder, Parameter> InputLowPassParameters;
    typedef ButterworthModel<kInputFilterOrder, Parameter> InputLowPassModel;
    typedef ButterworthState<kInputFilterOrder, Parameter> InputLowPassState;
    typedef ButterworthFilter<kInputFilterOrder, Accumulator, Sample, Accumulator> InputLowPassFilter;

    typedef ButterworthParameters<kOutputFilterOrder, Parameter> OutputLowPassParameters;
    typedef ButterworthModel<kOutputFilterOrder, Parameter> OutputLowPassModel;
    typedef ButterworthState<kOutputFilterOrder, Parameter> OutputLowPassState;
    typedef ButterworthFilter<kOutputFilterOrder, Accumulator, Accumulator, Sample> OutputLowPassFilter;

    typedef calfmoo::utils::RingBuffer<Sample, Index> SampleBuffer;
    typedef calfmoo::utils::RingBuffer<Accumulator, Index> AccumulatorBuffer;

    struct InputStage {
        InputLowPassState lowpass_state;
        InputLowPassFilter lowpass_filter;
        AccumulatorBuffer lowpass_output_buffer;

        InputResampler resampler;
        AccumulatorBuffer resampler_output_buffer;
    };

    struct OutputStage {
        AccumulatorBuffer chip_output_buffer;

        OutputResampler resampler;
        AccumulatorBuffer resampler_output_buffer;

        OutputLowPassState lowpass_state;
        OutputLowPassFilter lowpass_filter;
        SampleBuffer lowpass_output_buffer;
    };

protected:
    ChipFixed chip_fixed_;
    ChipFloat chip_float_;
    ChipIdeal chip_ideal_;
    ChipBase* chip_;
    Engine chip_engine_;

    SampleRate sample_rate_;
    Index block_size_;

    Slider dry_;
    Slider wet_;

    InputLowPassParameters input_lowpass_params_;
    InputLowPassModel input_lowpass_model_;

    OutputLowPassParameters output_lowpass_params_;
    OutputLowPassModel output_lowpass_model_;

    InputStage input_stages_[kInputChannelCount];
    OutputStage output_stages_[kOutputChannelCount];

    mutable std::recursive_mutex mutex_;
    volatile bool running_;

public:
    Plugin();
    ~Plugin();

    Register ReadRegister(Address address);
    void WriteRegister(Address address, Register data);

    Slider ReadParameter(Param index);
    void WriteParameter(Param index, Slider slider);

    void Start(SampleRate sample_rate, Index block_size);
    void Process(Sample** inputs, Sample** outputs, Index sample_count);
    void Stop();

    bool IsRunning()
    {
        std::scoped_lock lock(mutex_);
        return running_;
    }

    Engine GetEngine() const
    {
        std::scoped_lock lock(mutex_);
        return chip_engine_;
    }

protected:
    void ResampleInputs(Index length, Sample* inputs[]);
    void ResampleOutputs(Index length, Sample* outputs[]);
    void SelectEngine(Engine engine);

protected:
    template <typename Buffer>
    static void ReallocBuffer(Buffer& buffer, Index length);

    template <typename Buffer>
    static void DeallocBuffer(Buffer& buffer);

public:
    static Register SliderToGainData(Slider slider);
    static Slider GainDataToSlider(Register data);
    static Register SliderToCoeffData(Slider slider);
    static Slider CoeffDataToSlider(Register data);
    static Register SliderToTapData(Slider slider);
    static Slider TapDataToSlider(Register data);
    static Parameter SliderToFreq(Slider slider);
    static Slider FreqToSlider(Parameter freq);
};

// ============================================================================

#pragma pack(pop)

}  // namespace plugin

#endif  // _PLUGIN_HPP_
