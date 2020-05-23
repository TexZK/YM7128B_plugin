#ifndef _VSTPLUGIN_HPP_
#define _VSTPLUGIN_HPP_

#include "Plugin.hpp"
#include "audioeffectx.h"

#include <cmath>
#include <mutex>

// ============================================================================

namespace plugin {
namespace vst2 {

#pragma pack(push)
#pragma pack()

// ============================================================================

inline void PrintCoeff(char text[], Gain gain)
{
    if (gain == 0) {
        vst_strncpy(text, "-oo", kVstMaxParamStrLen);
    }
    else {
        double db = ToDecibels(std::abs(gain));
        snprintf(text, kVstMaxParamStrLen, "%.1f", db);
    }
}

inline void PrintGain(char text[], Gain gain)
{
    if (gain == 0) {
        vst_strncpy(text, "-oo", kVstMaxParamStrLen);
    }
    else {
        double db = ToDecibels(std::abs(gain));
        int n = (db < 0) ? int(db - 0.5f) : int(db + 0.5f);
        snprintf(text, kVstMaxParamStrLen, "%.0f", db);
    }
}

// ----------------------------------------------------------------------------

inline void PrintTap(char text[], Tap tap)
{
    double ms = (tap * 1000.0) / static_cast<double>(YM7128B_Input_Rate);
    snprintf(text, kVstMaxParamStrLen, "%.1f", ms);
}

// ----------------------------------------------------------------------------

inline void PrintFrequency(char text[], double frequency)
{
    snprintf(text, kVstMaxParamStrLen, "%.0f", frequency);
}

// ----------------------------------------------------------------------------

inline void PrintSignedDecibels(char label[], Gain gain)
{
    vst_strncpy(label, (gain < 0 ? "dB-" : "dB+"), kVstMaxParamStrLen);
}

// ============================================================================

class Plugin : public AudioEffectX
{
protected:
    typedef uint_fast64_t ParamMask;
    typedef ::plugin::Plugin Wrapped;

    char program_name_[kVstMaxProgNameLen + 1];

    mutable std::recursive_mutex mutex_;
    Slider cached_sliders_[to_underlying(Param::kCount)];
    ParamMask cached_mask_;

    Wrapped wrapped_;

protected:
    void ApplyCachedParameters();

public:
    void setProgram(VstInt32 program);

    void setProgramName(char *name)
    {
        vst_strncpy(program_name_, name, kVstMaxProgNameLen);
    }

    void getProgramName(char *name)
    {
        vst_strncpy(name, program_name_, kVstMaxProgNameLen);
    }

    bool getProgramNameIndexed(VstInt32 category, VstInt32 index, char *text)
    {
        (void)category;

        if (index >= 0 && index < static_cast<VstInt32>(Program::kCount)) {
            vst_strncpy(text, kProgramNames[index], kVstMaxProgNameLen);
            return true;
        }
        else {
            return false;
        }
    }

    void setParameter(VstInt32 index, float slider)
    {
        if (index < static_cast<VstInt32>(Param::kCount)) {
            std::scoped_lock lock(mutex_);
            cached_mask_ |= static_cast<ParamMask>(1) << index;
            cached_sliders_[index] = slider;
        }
    }

    float getParameter(VstInt32 index)
    {
        if (index < static_cast<VstInt32>(Param::kCount)) {
            std::scoped_lock lock(mutex_);
            return static_cast<float>(cached_sliders_[index]);
        }
        return 0;
    }

    void getParameterLabel(VstInt32 index, char *label);
    void getParameterDisplay(VstInt32 index, char *text);

    void getParameterName(VstInt32 index, char *text)
    {
        if (index < to_underlying(Param::kCount)) {
            vst_strncpy(text, kParamNames[index], kVstMaxLabelLen);
        }
    }

    bool getEffectName(char *name)
    {
        vst_strncpy(name, "YM7128B", kVstMaxEffectNameLen);
        return true;
    }

    bool getVendorString(char *text)
    {
        vst_strncpy(text, "Andrea Zoppi", kVstMaxVendorStrLen);
        return true;
    }

    bool getProductString(char *text)
    {
        vst_strncpy(text, "YM7128B", kVstMaxProductStrLen);
        return true;
    }

    VstInt32 getVendorVersion()
    {
        return 0x0004;
    }

    VstInt32 startProcess()
    {
        wrapped_.Start(static_cast<Wrapped::SampleRate>(getSampleRate()), getBlockSize());
        return 0;
    }

    void processReplacing(float **inputs, float **outputs, VstInt32 sample_count)
    {
        ApplyCachedParameters();
        wrapped_.Process(inputs, outputs, static_cast<Wrapped::Index>(sample_count));
    }

    VstInt32 stopProcess()
    {
        wrapped_.Stop();
        return 0;
    }

public:
    Plugin(audioMasterCallback audioMaster) :
        AudioEffectX(audioMaster,
                     static_cast<VstInt32>(Program::kCount),
                     static_cast<VstInt32>(Param::kCount))
    {
        setNumInputs(YM7128B_InputChannel_Count);
        setNumOutputs(YM7128B_OutputChannel_Count);
        setUniqueID(0x89777128);
        canProcessReplacing();
        setProgram(curProgram);
    }

    ~Plugin()
    {
        ;
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace vst2
}  // namespace plugin

#endif  // _VSTPLUGIN_HPP_
