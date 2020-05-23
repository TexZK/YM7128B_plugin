#include "VstPlugin.hpp"

// ============================================================================

namespace plugin {
namespace vst2 {

// ============================================================================

template <typename T>
static inline T guard_return(std::recursive_mutex& mtx, T const& target)
{
    std::scoped_lock lock(mtx);
    return target;
}

// ============================================================================

void Plugin::ApplyCachedParameters()
{
    for (;;) {
        std::scoped_lock lock(mutex_);
        unsigned long index;
        if (FindFirstBit(index, cached_mask_)) {
            Param param = static_cast<Param>(index);
            wrapped_.WriteParameter(param, cached_sliders_[index]);
            cached_mask_ -= static_cast<ParamMask>(1) << index;
        }
        else {
            break;
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::setProgram(VstInt32 program)
{
    if ((program < 0) || (program >= static_cast<VstInt32>(Program::kCount))) {
        program = 0;
    }

    std::scoped_lock lock(mutex_);

    for (Address i = 0; i < YM7128B_Address_Max; ++i) {
        wrapped_.WriteRegister(i, kProgramRegisters[program][i]);
    }
    for (unsigned i = 0; i < static_cast<unsigned>(Param::kCount); ++i) {
        cached_sliders_[i] = wrapped_.ReadParameter(static_cast<Param>(i));
    }

    curProgram = program;
}

// ----------------------------------------------------------------------------

void Plugin::getParameterDisplay(VstInt32 index, char* text)
{
    switch (index)
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
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Register data = Wrapped::SliderToGainData(slider);
            Gain value = YM7128B_RegisterToGainFloat(data);
            PrintGain(text, value);
            break;
        }

        case YM7128B_Reg_C0:
        case YM7128B_Reg_C1: {
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Register data = Wrapped::SliderToCoeffData(slider);
            Gain value = YM7128B_RegisterToCoeffFloat(data);
            PrintCoeff(text, value);
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
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Register data = Wrapped::SliderToTapData(slider);
            Tap tap = YM7128B_Tap_Table[data];
            PrintTap(text, tap);
            break;
        }

        case static_cast<VstInt32>(Param::kInputCutoff):
        case static_cast<VstInt32>(Param::kOutputCutoff): {
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Wrapped::Parameter freq = Wrapped::SliderToFreq(slider);
            PrintFrequency(text, freq);
            break;
        }

        case static_cast<VstInt32>(Param::kDry):
        case static_cast<VstInt32>(Param::kWet): {
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Gain gain = SliderToVolumeExp(slider, DecibelRange::k60dB);
            PrintGain(text, gain);
            break;
        }

        case static_cast<VstInt32>(Param::kEngine): {
            Engine engine = wrapped_.GetEngine();
            vst_strncpy(text, kEngineNames[to_underlying(engine)], kVstMaxParamStrLen);
            break;
        }

        default: {
            vst_strncpy(text, "???", kVstMaxParamStrLen);
            break;
        }
    }
}

// ----------------------------------------------------------------------------

void Plugin::getParameterLabel(VstInt32 index, char *label)
{
    switch (index)
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
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Register data = Wrapped::SliderToGainData(slider);
            Gain value = YM7128B_RegisterToGainFloat(data);
            PrintSignedDecibels(label, value);
            break;
        }

        case YM7128B_Reg_C0:
        case YM7128B_Reg_C1: {
            Slider slider = guard_return(mutex_, cached_sliders_[index]);
            Register data = Wrapped::SliderToCoeffData(slider);
            Gain value = YM7128B_RegisterToCoeffFloat(data);
            PrintSignedDecibels(label, value);
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
            vst_strncpy(label, "ms", kVstMaxParamStrLen);
            break;
        }

        case static_cast<VstInt32>(Param::kInputCutoff):
        case static_cast<VstInt32>(Param::kOutputCutoff): {
            vst_strncpy(label, "Hz", kVstMaxParamStrLen);
            break;
        }

        case static_cast<VstInt32>(Param::kDry):
        case static_cast<VstInt32>(Param::kWet): {
            vst_strncpy(label, "dB", kVstMaxParamStrLen);
            break;
        }

        case static_cast<VstInt32>(Param::kEngine): {
            vst_strncpy(label, "", kVstMaxParamStrLen);
            break;
        }

        default: {
            vst_strncpy(label, "???", kVstMaxParamStrLen);
            break;
        }
    }
}

// ============================================================================

}  // namespace vst2
}  // namespace plugin

// ============================================================================

AudioEffect* createEffectInstance(audioMasterCallback audio_master)
{
    return new plugin::vst2::Plugin(audio_master);
}
