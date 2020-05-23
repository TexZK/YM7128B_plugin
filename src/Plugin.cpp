#define NOMINMAX
#include "Plugin.hpp"

#include <algorithm>
#include <cmath>

// ============================================================================

namespace plugin {

#pragma pack()

// ============================================================================

char const* const kParamNames[to_underlying(Param::kCount)] =
{
    "GL1", "GL2", "GL3", "GL4", "GL5", "GL6", "GL7", "GL8",
    "GR1", "GR2", "GR3", "GR4", "GR5", "GR6", "GR7", "GR8",
    "VM", "VC", "VL", "VR",
    "C0", "C1",
    "T0", "T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8",
    "LPI", "LPO",
    "DRY", "WET",
    "Engine",
};

char const* const kProgramNames[to_underlying(Program::kCount)] =
{
    "Off",
    "Direct",
    "Dune / Arrakis",
    "Dune / Baghdad",
    "Dune / Morning",
    "Dune / Sequence",
    "Dune / Sietch",
    "Dune / War Song",
    "Dune / Water",
    "Dune / Worm Intro",
    "Dune / Wormsuit",
};

char const* const kEngineNames[to_underlying(Engine::kCount)] =
{
    "Fixed",
    "Float",
    "Ideal",
};

Register kProgramRegisters[to_underlying(Program::kCount)][YM7128B_Reg_Count] =
{
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    },
    {
        0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x3F, 0x00, 0x3F, 0x3F,
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    },
    {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1A, 0x1A,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    },
    {
        0x1F, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x33, 0x00,
        0x00, 0x1D, 0x00, 0x19, 0x00, 0x15, 0x00, 0x11,
        0x1D, 0x1D, 0x1D, 0x1D,
        0x13, 0x13,
        0x06, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
    },
    {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1B, 0x1B,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    },
    {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1C, 0x1C,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    },
    {
        0x1F, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x33, 0x00,
        0x00, 0x1D, 0x00, 0x19, 0x00, 0x15, 0x00, 0x11,
        0x1D, 0x1D, 0x1D, 0x1D,
        0x13, 0x13,
        0x06, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
    },
    {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1C, 0x1C,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    },
    {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1A, 0x1A,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    },
    {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x18, 0x18,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    },
    {
        0x18, 0x00, 0x1A, 0x00, 0x1C, 0x00, 0x1E, 0x00,
        0x00, 0x19, 0x00, 0x1B, 0x00, 0x1D, 0x00, 0x1F,
        0x1B, 0x1F, 0x17, 0x17,
        0x12, 0x08,
        0x1F, 0x07, 0x0A, 0x0D, 0x10, 0x13, 0x16, 0x19, 0x1C,
    },
};


double const kDecibelSliderArgCoeff[to_underlying(DecibelRange::kCount)] =
{
     5.757,
     6.908,
     8.059,
     9.210,
    10.360,
    11.510,
};

double const kDecibelSliderExpCoeff[to_underlying(DecibelRange::kCount)] =
{
    0.003162300,
    0.001000000,
    0.000316230,
    0.000100000,
    0.000031623,
    0.000010000,
};

int const kDecibelSliderPowCoeff[to_underlying(DecibelRange::kCount)] =
{
    3, 4, 5, 6, 6, 7,
};

// ----------------------------------------------------------------------------

static char const kHexChars[] = "0123456789ABCDEF";

// ============================================================================

Plugin::Plugin() :
    chip_fixed_(),
    chip_float_(),
    chip_ideal_(),
    chip_(&chip_fixed_),
    chip_engine_(Engine::kFixed),

    sample_rate_(44100),
    block_size_(256),

    dry_(1),
    wet_(1),

    running_(false)
{
    input_lowpass_params_.SetFrequency(15000);
    output_lowpass_params_.SetFrequency(15000);
}

// ----------------------------------------------------------------------------

Plugin::~Plugin()
{
    Stop();

    std::scoped_lock lock(mutex_);

    for (unsigned i = 0; i < kInputChannelCount; ++i) {
        InputStage& stage = input_stages_[i];
        DeallocBuffer(stage.lowpass_output_buffer);
        DeallocBuffer(stage.resampler_output_buffer);
    }

    for (unsigned i = 0; i < kOutputChannelCount; ++i) {
        OutputStage& stage = output_stages_[i];
        DeallocBuffer(stage.chip_output_buffer);
        DeallocBuffer(stage.resampler_output_buffer);
        DeallocBuffer(stage.lowpass_output_buffer);
    }
}

// ----------------------------------------------------------------------------

Register Plugin::ReadRegister(Address address)
{
    std::scoped_lock lock(mutex_);

    return chip_->Read(address);
}

// ----------------------------------------------------------------------------

void Plugin::WriteRegister(Address address, Register data)
{
    std::scoped_lock lock(mutex_);

    chip_fixed_.Write(address, data);
    chip_float_.Write(address, data);
    chip_ideal_.Write(address, data);
}

// ----------------------------------------------------------------------------

Slider Plugin::ReadParameter(Param index)
{
    std::scoped_lock lock(mutex_);

    switch (static_cast<unsigned>(index))
    {
        case YM7128B_Reg_GL1:
        case YM7128B_Reg_GL2:
        case YM7128B_Reg_GL3:
        case YM7128B_Reg_GL4:
        case YM7128B_Reg_GL5:
        case YM7128B_Reg_GL6:
        case YM7128B_Reg_GL7:
        case YM7128B_Reg_GL8:
        case YM7128B_Reg_GR1:
        case YM7128B_Reg_GR2:
        case YM7128B_Reg_GR3:
        case YM7128B_Reg_GR4:
        case YM7128B_Reg_GR5:
        case YM7128B_Reg_GR6:
        case YM7128B_Reg_GR7:
        case YM7128B_Reg_GR8:
        case YM7128B_Reg_VM:
        case YM7128B_Reg_VC:
        case YM7128B_Reg_VL:
        case YM7128B_Reg_VR: {
            Address address = static_cast<Address>(index);
            Register data = chip_->Read(address);
            Slider slider = GainDataToSlider(data);
            return slider;
        }

        case YM7128B_Reg_C0:
        case YM7128B_Reg_C1: {
            Address address = static_cast<Address>(index);
            Register data = chip_->Read(address);
            Slider slider = CoeffDataToSlider(data);
            return slider;
        }

        case YM7128B_Reg_T0:
        case YM7128B_Reg_T1:
        case YM7128B_Reg_T2:
        case YM7128B_Reg_T3:
        case YM7128B_Reg_T4:
        case YM7128B_Reg_T5:
        case YM7128B_Reg_T6:
        case YM7128B_Reg_T7:
        case YM7128B_Reg_T8: {
            Address address = static_cast<Address>(index);
            Register data = chip_->Read(address);
            Slider slider = TapDataToSlider(data);
            return slider;
        }

        case static_cast<unsigned>(Param::kInputCutoff): {
            Parameter freq = input_lowpass_params_.GetFrequency();
            Slider slider = FreqToSlider(freq);
            return slider;
        }

        case static_cast<unsigned>(Param::kOutputCutoff): {
            Parameter freq = output_lowpass_params_.GetFrequency();
            Slider slider = FreqToSlider(freq);
            return slider;
        }

        case static_cast<unsigned>(Param::kDry): {
            Slider slider = VolumeToSliderExp(dry_, DecibelRange::k60dB);
            return slider;
        }
        case static_cast<unsigned>(Param::kWet): {
            Slider slider = VolumeToSliderExp(wet_, DecibelRange::k60dB);
            return slider;
        }

        case static_cast<unsigned>(Param::kEngine): {
            Slider slider = IntegralToSlider<Engine, Engine::kIdeal>(chip_engine_);
            return slider;
        }

        default: {
            return 0;
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::WriteParameter(Param index, Slider slider)
{
    std::scoped_lock lock(mutex_);

    switch (static_cast<unsigned>(index))
    {
        case YM7128B_Reg_GL1:
        case YM7128B_Reg_GL2:
        case YM7128B_Reg_GL3:
        case YM7128B_Reg_GL4:
        case YM7128B_Reg_GL5:
        case YM7128B_Reg_GL6:
        case YM7128B_Reg_GL7:
        case YM7128B_Reg_GL8:
        case YM7128B_Reg_GR1:
        case YM7128B_Reg_GR2:
        case YM7128B_Reg_GR3:
        case YM7128B_Reg_GR4:
        case YM7128B_Reg_GR5:
        case YM7128B_Reg_GR6:
        case YM7128B_Reg_GR7:
        case YM7128B_Reg_GR8:
        case YM7128B_Reg_VM:
        case YM7128B_Reg_VC:
        case YM7128B_Reg_VL:
        case YM7128B_Reg_VR: {
            Address address = static_cast<Address>(index);
            Register data = SliderToGainData(slider);
            chip_fixed_.Write(address, data);
            chip_float_.Write(address, data);
            chip_ideal_.Write(address, data);
            break;
        }

        case YM7128B_Reg_C0:
        case YM7128B_Reg_C1: {
            Address address = static_cast<Address>(index);
            Register data = SliderToCoeffData(slider);
            chip_fixed_.Write(address, data);
            chip_float_.Write(address, data);
            chip_ideal_.Write(address, data);
            break;
        }

        case YM7128B_Reg_T0:
        case YM7128B_Reg_T1:
        case YM7128B_Reg_T2:
        case YM7128B_Reg_T3:
        case YM7128B_Reg_T4:
        case YM7128B_Reg_T5:
        case YM7128B_Reg_T6:
        case YM7128B_Reg_T7:
        case YM7128B_Reg_T8: {
            Address address = static_cast<Address>(index);
            Register data = SliderToTapData(slider);
            chip_fixed_.Write(address, data);
            chip_float_.Write(address, data);
            chip_ideal_.Write(address, data);
            break;
        }

        case static_cast<unsigned>(Param::kInputCutoff): {
            Parameter frequency = SliderToFreq(slider);
            Parameter max_lowpass_frequency = std::min(20000.0f, sample_rate_ * 0.95f);
            input_lowpass_params_.SetFrequency(std::min(frequency, max_lowpass_frequency));
            input_lowpass_model_ = input_lowpass_params_;
            break;
        }
        case static_cast<unsigned>(Param::kOutputCutoff): {
            Parameter frequency = SliderToFreq(slider);
            Parameter max_lowpass_frequency = std::min(20000.0f, sample_rate_ * 0.95f);
            output_lowpass_params_.SetFrequency(std::min(frequency, max_lowpass_frequency));
            output_lowpass_model_ = output_lowpass_params_;
            break;
        }

        case static_cast<unsigned>(Param::kDry): {
            dry_ = SliderToVolumeExp(slider, DecibelRange::k60dB);
            break;
        }
        case static_cast<unsigned>(Param::kWet): {
            wet_ = SliderToVolumeExp(slider, DecibelRange::k60dB);
            break;
        }

        case static_cast<unsigned>(Param::kEngine): {
            Engine engine = SliderToIntegral<Engine, Engine::kIdeal>(slider);
            SelectEngine(engine);
            break;
        }

        default: {
            break;
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::Start(SampleRate sample_rate, Index block_size)
{
    std::scoped_lock lock(mutex_);

    chip_->Start(static_cast<ChipFloat::Index>(sample_rate), block_size);

    sample_rate_ = sample_rate;
    block_size_ = block_size;
    Index buffer_size = (block_size * 3) / 2;
    Parameter max_lowpass_frequency = static_cast<Parameter>(std::min(20000.0, sample_rate * 0.95));

    input_lowpass_params_.SetBehavior(InputLowPassParameters::kBehaviorLowPass);
    input_lowpass_params_.SetSampleRate(sample_rate);
    if (max_lowpass_frequency < input_lowpass_params_.GetFrequency()) {
        input_lowpass_params_.SetFrequency(max_lowpass_frequency);
    }
    input_lowpass_params_.UpdateResonance();
    input_lowpass_model_ = input_lowpass_params_;

    output_lowpass_params_.SetBehavior(OutputLowPassParameters::kBehaviorLowPass);
    output_lowpass_params_.SetSampleRate(sample_rate);
    if (max_lowpass_frequency < output_lowpass_params_.GetFrequency()) {
        output_lowpass_params_.SetFrequency(max_lowpass_frequency);
    }
    output_lowpass_params_.UpdateResonance();
    output_lowpass_model_ = output_lowpass_params_;

    for (unsigned i = 0; i < kInputChannelCount; ++i) {
        InputStage& stage = input_stages_[i];

        stage.lowpass_state.Clear();
        stage.lowpass_filter.SetModel(&input_lowpass_model_);
        stage.lowpass_filter.SetState(&stage.lowpass_state);
        ReallocBuffer(stage.lowpass_output_buffer, buffer_size);

        stage.resampler.Start(static_cast<InputResampler::Rate>(sample_rate),
                              static_cast<InputResampler::Rate>(YM7128B_Input_Rate));
        ReallocBuffer(stage.resampler_output_buffer, buffer_size);
    }

    for (unsigned i = 0; i < kOutputChannelCount; ++i) {
        OutputStage& stage = output_stages_[i];

        ReallocBuffer(stage.chip_output_buffer, buffer_size);

        stage.resampler.Start(static_cast<OutputResampler::Rate>(YM7128B_Output_Rate),
                              static_cast<OutputResampler::Rate>(sample_rate));
        ReallocBuffer(stage.resampler_output_buffer, buffer_size);

        stage.lowpass_state.Clear();
        stage.lowpass_filter.SetModel(&output_lowpass_model_);
        stage.lowpass_filter.SetState(&stage.lowpass_state);
        ReallocBuffer(stage.lowpass_output_buffer, buffer_size);
    }

    running_ = true;
}

// ----------------------------------------------------------------------------

void Plugin::Process(Sample** inputs, Sample** outputs, Index sample_count)
{
    std::scoped_lock lock(mutex_);
    if (!running_) {
        return;
    }

    ResampleInputs(sample_count, inputs);

    AccumulatorBuffer& inbuf = input_stages_[0].resampler_output_buffer;
    AccumulatorBuffer& outbuf1 = output_stages_[0].chip_output_buffer;
    AccumulatorBuffer& outbuf2 = output_stages_[1].chip_output_buffer;
    Accumulator* chip_outputs[kOutputChannelCount];
    Index ovs = (chip_engine_ == Engine::kIdeal) ? 1 : YM7128B_Oversampling;
    Index count;

    while (inbuf.GetUsed() && outbuf1.GetFree() && outbuf2.GetFree()) {
        count = outbuf1.GetEnqueueContiguous() / ovs;
        count = inbuf.TrimDequeueContiguous(count);

        chip_outputs[0] = outbuf1.GetTailPointer();
        chip_outputs[1] = outbuf2.GetTailPointer();
        (*chip_)(count, inbuf.GetHeadPointer(), chip_outputs);

        inbuf.SkipHead(count);
        outbuf1.SkipTail(count * ovs);
        outbuf2.SkipTail(count * ovs);
    }

    ResampleOutputs(sample_count, outputs);

    Slider dry_gain = dry_;
    Slider wet_gain = wet_;
    for (unsigned channel = 0; channel < kOutputChannelCount; ++channel) {
        Sample* channel_outputs = outputs[channel];
        for (Index i = 0; i < sample_count; ++i) {
            double dry_signal = inputs[0][i] * dry_gain;
            double wet_signal = channel_outputs[i] * wet_gain;
            double output = dry_signal + wet_signal;
            channel_outputs[i] = static_cast<Sample>(output);
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::Stop()
{
    std::scoped_lock lock(mutex_);
    running_ = false;

    chip_->Stop();

    for (unsigned i = 0; i < kInputChannelCount; ++i) {
        InputStage& stage = input_stages_[i];

        DeallocBuffer(stage.lowpass_output_buffer);

        stage.resampler.Stop();
        DeallocBuffer(stage.resampler_output_buffer);
    }

    for (unsigned i = 0; i < kOutputChannelCount; ++i) {
        OutputStage& stage = output_stages_[i];

        DeallocBuffer(stage.chip_output_buffer);

        stage.resampler.Stop();
        DeallocBuffer(stage.resampler_output_buffer);

        DeallocBuffer(stage.lowpass_output_buffer);
    }
}

// ----------------------------------------------------------------------------

void Plugin::ResampleInputs(Index length, Sample* inputs[])
{
    if (chip_engine_ == Engine::kIdeal) {
        for (unsigned i = 0; i < kInputChannelCount; ++i) {
            SampleBuffer samples(length, inputs[i], length);
            InputStage& stage = input_stages_[i];
            AccumulatorBuffer& lowpassed = stage.lowpass_output_buffer;
            AccumulatorBuffer& resampled = stage.resampler_output_buffer;
            InputLowPassFilter& lowpass = stage.lowpass_filter;
            InputResampler& resample = stage.resampler;
            lowpass(samples, lowpassed);
            SampleBuffer::Move(lowpassed, resampled);
        }
    }
    else {
        for (unsigned i = 0; i < kInputChannelCount; ++i) {
            SampleBuffer samples(length, inputs[i], length);
            InputStage& stage = input_stages_[i];
            AccumulatorBuffer& lowpassed = stage.lowpass_output_buffer;
            AccumulatorBuffer& resampled = stage.resampler_output_buffer;
            InputLowPassFilter& lowpass = stage.lowpass_filter;
            InputResampler& resample = stage.resampler;
            lowpass(samples, lowpassed);
            resample(lowpassed, resampled);
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::ResampleOutputs(Index length, Sample* outputs[])
{
    if (chip_engine_ == Engine::kIdeal) {
        for (unsigned i = 0; i < kOutputChannelCount; ++i) {
            SampleBuffer samples(length, outputs[i]);
            OutputStage& stage = output_stages_[i];
            AccumulatorBuffer& reflected = stage.chip_output_buffer;
            AccumulatorBuffer& resampled = stage.resampler_output_buffer;
            SampleBuffer& lowpassed = stage.lowpass_output_buffer;
            OutputLowPassFilter& lowpass = stage.lowpass_filter;
            OutputResampler& resample = stage.resampler;
            SampleBuffer::Move(reflected, resampled);
            lowpass(resampled, lowpassed);
            SampleBuffer::Move(lowpassed, samples);
        }
    }
    else {
        for (unsigned i = 0; i < kOutputChannelCount; ++i) {
            SampleBuffer samples(length, outputs[i]);
            OutputStage& stage = output_stages_[i];
            AccumulatorBuffer& reflected = stage.chip_output_buffer;
            AccumulatorBuffer& resampled = stage.resampler_output_buffer;
            SampleBuffer& lowpassed = stage.lowpass_output_buffer;
            OutputLowPassFilter& lowpass = stage.lowpass_filter;
            OutputResampler& resample = stage.resampler;
            resample(reflected, resampled);
            lowpass(resampled, lowpassed);
            SampleBuffer::Move(lowpassed, samples);
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::SelectEngine(Engine engine)
{
    if (engine != chip_engine_ && engine < Engine::kCount) {
        bool running = running_;
        Stop();

        ChipBase* engines[] = { &chip_fixed_, &chip_float_, &chip_ideal_ };
        chip_ = engines[to_underlying(engine)];
        chip_engine_ = engine;

        if (running) {
            Start(sample_rate_, block_size_);
        }
    }
}

// ----------------------------------------------------------------------------

template <typename Buffer>
void Plugin::ReallocBuffer(Buffer& buffer, Index length)
{
    Buffer::Item* samples = buffer.GetBuffer();
    if (buffer.GetLength() != length) {
        delete[] samples;
        samples = new Buffer::Item[length];
    }
    buffer.Initialize(length, samples);
}

// ----------------------------------------------------------------------------

template <typename Buffer>
void Plugin::DeallocBuffer(Buffer& buffer)
{
    Buffer::Item* samples = buffer.GetBuffer();
    delete[] samples;
    buffer.Initialize(0, nullptr);
}

// ----------------------------------------------------------------------------

Register Plugin::SliderToGainData(Slider slider)
{
    Register data = SliderToIntegral<Register, YM7128B_Gain_Data_Mask>(slider);
    if (data < YM7128B_Gain_Data_Count / 2) {
        data = (YM7128B_Gain_Data_Count / 2 - 1) - data;
    }
    return data;
}

// ----------------------------------------------------------------------------

Slider Plugin::GainDataToSlider(Register data)
{
    if (data < YM7128B_Gain_Data_Count / 2) {
        data = (YM7128B_Gain_Data_Count / 2 - 1) - data;
    }
    Slider const k = (1 / static_cast<Slider>(YM7128B_Gain_Data_Mask));
    Slider slider = data * k;
    return slider;
}

// ----------------------------------------------------------------------------

Register Plugin::SliderToCoeffData(Slider slider)
{
    Register data = SliderToIntegral<Register, YM7128B_Coeff_Value_Mask>(slider);
    data = (data + (YM7128B_Coeff_Value_Count / 2)) & YM7128B_Coeff_Value_Mask;
    return data;
}

// ----------------------------------------------------------------------------

Slider Plugin::CoeffDataToSlider(Register data)
{
    data = (data - (YM7128B_Coeff_Value_Count / 2)) & YM7128B_Coeff_Value_Mask;
    Slider const k = (1 / static_cast<Slider>(YM7128B_Coeff_Value_Mask));
    Slider slider = data * k;
    return slider;
}

// ----------------------------------------------------------------------------

Register Plugin::SliderToTapData(Slider slider)
{
    Register data = SliderToIntegral<Register, YM7128B_Tap_Value_Mask>(slider);
    return data;
}

// ----------------------------------------------------------------------------

Slider Plugin::TapDataToSlider(Register data)
{
    Slider const k = (1 / static_cast<Slider>(YM7128B_Tap_Value_Mask));
    Slider slider = data * k;
    return slider;
}

// ----------------------------------------------------------------------------

Plugin::Parameter Plugin::SliderToFreq(Slider slider)
{
    Parameter freq = Lerp<Parameter>(4000, 20000, static_cast<Parameter>(slider));
    return freq;
}

// ----------------------------------------------------------------------------

Slider Plugin::FreqToSlider(Parameter freq)
{
    Slider slider = LerpInv<Slider>(4000, 20000, static_cast<Slider>(freq));
    return slider;
}

// ============================================================================

}  // namespace plugin
