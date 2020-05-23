#include "LadspaPlugin.hpp"

// ============================================================================

namespace plugin {
namespace ladspa {

#pragma pack()

// ============================================================================

LADSPA_PortDescriptor const kPortDescriptorSlider =
    (LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL);

LADSPA_PortDescriptor const kPortDescriptorInput =
    (LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO);

LADSPA_PortDescriptor const kPortDescriptorOutput =
    (LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO);

LADSPA_PortDescriptor const kPortDescriptors[kPortCount] =
{
    kPortDescriptorSlider,  // kRegGL1
    kPortDescriptorSlider,  // kRegGL2
    kPortDescriptorSlider,  // kRegGL3
    kPortDescriptorSlider,  // kRegGL4
    kPortDescriptorSlider,  // kRegGL5
    kPortDescriptorSlider,  // kRegGL6
    kPortDescriptorSlider,  // kRegGL7
    kPortDescriptorSlider,  // kRegGL8

    kPortDescriptorSlider,  // kRegGR1
    kPortDescriptorSlider,  // kRegGR2
    kPortDescriptorSlider,  // kRegGR3
    kPortDescriptorSlider,  // kRegGR4
    kPortDescriptorSlider,  // kRegGR5
    kPortDescriptorSlider,  // kRegGR6
    kPortDescriptorSlider,  // kRegGR7
    kPortDescriptorSlider,  // kRegGR8

    kPortDescriptorSlider,  // kRegVM
    kPortDescriptorSlider,  // kRegVC
    kPortDescriptorSlider,  // kRegVL
    kPortDescriptorSlider,  // kRegVR

    kPortDescriptorSlider,  // kRegC0
    kPortDescriptorSlider,  // kRegC1

    kPortDescriptorSlider,  // kRegT0
    kPortDescriptorSlider,  // kRegT1
    kPortDescriptorSlider,  // kRegT2
    kPortDescriptorSlider,  // kRegT3
    kPortDescriptorSlider,  // kRegT4
    kPortDescriptorSlider,  // kRegT5
    kPortDescriptorSlider,  // kRegT6
    kPortDescriptorSlider,  // kRegT7
    kPortDescriptorSlider,  // kRegT8

    kPortDescriptorSlider,  // kParamInputCutoff
    kPortDescriptorSlider,  // kParamOutputCutoff

    kPortDescriptorSlider,  // kParamDry
    kPortDescriptorSlider,  // kParamWet

    kPortDescriptorSlider,  // kParamEngine

    kPortDescriptorInput,  // audio input (mono)

    kPortDescriptorOutput,  // audio output left
    kPortDescriptorOutput,  // audio output right
};

// ----------------------------------------------------------------------------

static LADSPA_Data const kBoundMargin = static_cast<LADSPA_Data>(0.01);

LADSPA_PortRangeHint const kPortRangeHintGain = {
    (LADSPA_HINT_BOUNDED_BELOW |
     LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_DEFAULT_0),
    -62,
    +62,
};

LADSPA_PortRangeHint const kPortRangeHintCoeff = {
    (LADSPA_HINT_BOUNDED_BELOW |
     LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_DEFAULT_0),
    -1,
    +1,
};

LADSPA_PortRangeHint const kPortRangeHintTap = {
    (LADSPA_HINT_BOUNDED_BELOW |
     LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_DEFAULT_MINIMUM),
    0,
    100,
};

LADSPA_PortRangeHint const kPortRangeHintFilter = {
    (LADSPA_HINT_BOUNDED_BELOW |
     LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_DEFAULT_HIGH),
    4000,
    20000,
};

LADSPA_PortRangeHint const kPortRangeHintDryWet = {
    (LADSPA_HINT_BOUNDED_BELOW |
     LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_DEFAULT_MAXIMUM),
    0,
    1,
};

LADSPA_PortRangeHint const kPortRangeHintEngine = {
    (LADSPA_HINT_BOUNDED_BELOW |
     LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_DEFAULT_MINIMUM |
     LADSPA_HINT_INTEGER),
    1 - kBoundMargin,
    static_cast<LADSPA_Data>(Engine::kCount) + kBoundMargin,
};

LADSPA_PortRangeHint const kPortRangeHintSound = {
    0,
    YM7128B_Float_Min,
    YM7128B_Float_Max,
};

LADSPA_PortRangeHint const kPortRangeHints[kPortCount] =
{
    kPortRangeHintGain,  // kRegGL1
    kPortRangeHintGain,  // kRegGL2
    kPortRangeHintGain,  // kRegGL3
    kPortRangeHintGain,  // kRegGL4
    kPortRangeHintGain,  // kRegGL5
    kPortRangeHintGain,  // kRegGL6
    kPortRangeHintGain,  // kRegGL7
    kPortRangeHintGain,  // kRegGL8

    kPortRangeHintGain,  // kRegGR1
    kPortRangeHintGain,  // kRegGR2
    kPortRangeHintGain,  // kRegGR3
    kPortRangeHintGain,  // kRegGR4
    kPortRangeHintGain,  // kRegGR5
    kPortRangeHintGain,  // kRegGR6
    kPortRangeHintGain,  // kRegGR7
    kPortRangeHintGain,  // kRegGR8

    kPortRangeHintGain,  // kRegVM
    kPortRangeHintGain,  // kRegVC
    kPortRangeHintGain,  // kRegVL
    kPortRangeHintGain,  // kRegVR

    kPortRangeHintCoeff,  // kRegC0
    kPortRangeHintCoeff,  // kRegC1

    kPortRangeHintTap,  // kRegT0
    kPortRangeHintTap,  // kRegT1
    kPortRangeHintTap,  // kRegT2
    kPortRangeHintTap,  // kRegT3
    kPortRangeHintTap,  // kRegT4
    kPortRangeHintTap,  // kRegT5
    kPortRangeHintTap,  // kRegT6
    kPortRangeHintTap,  // kRegT7
    kPortRangeHintTap,  // kRegT8

    kPortRangeHintFilter,  // kParamInputCutoff
    kPortRangeHintFilter,  // kParamOutputCutoff

    kPortRangeHintDryWet,  // kParamDry
    kPortRangeHintDryWet,  // kParamWet

    kPortRangeHintEngine,  // kParamEngine

    kPortRangeHintSound,  // input (mono)

    kPortRangeHintSound,  // output left
    kPortRangeHintSound,  // output right
};

// ----------------------------------------------------------------------------

LADSPA_Descriptor const kDescriptor =
{
    0x00123456,
    "YM7128B",
    (LADSPA_PROPERTY_REALTIME | LADSPA_PROPERTY_HARD_RT_CAPABLE),
    "YM7128B",
    "Andrea Zoppi",
    "(c) 2020 Andrea Zoppi. All rights reserved.",
    kPortCount,
    kPortDescriptors,
    kParamNames,
    kPortRangeHints,
    NULL,
    Instantiate,
    ConnectPort,
    Activate,
    Run,
    NULL,
    NULL,
    Deactivate,
    Cleanup,
};

// ============================================================================

LADSPA_Handle Instantiate(LADSPA_Descriptor const *descriptor, unsigned long sample_rate)
{
    Plugin* plugin = new Plugin(sample_rate);
    LADSPA_Handle instance = static_cast<LADSPA_Handle>(plugin);
    return instance;
}

// ----------------------------------------------------------------------------

void ConnectPort(LADSPA_Handle instance, unsigned long port, LADSPA_Data *data)
{
    Plugin *plugin = static_cast<Plugin *>(instance);
    plugin->ConnectPort(port, data);
}

// ----------------------------------------------------------------------------

void Activate(LADSPA_Handle instance)
{
    Plugin *plugin = static_cast<Plugin *>(instance);
    plugin->Activate();
}

// ----------------------------------------------------------------------------

void Run(LADSPA_Handle instance, unsigned long sample_count)
{
    Plugin *plugin = static_cast<Plugin *>(instance);
    plugin->Run(sample_count);
}

// ----------------------------------------------------------------------------

void Deactivate(LADSPA_Handle instance)
{
    Plugin *plugin = static_cast<Plugin *>(instance);
    plugin->Deactivate();
}

// ----------------------------------------------------------------------------

void Cleanup(LADSPA_Handle instance)
{
    Plugin *plugin = static_cast<Plugin *>(instance);
    delete plugin;
}

// ============================================================================

}  // namespace ladspa
}  // namespace plugin

// ============================================================================

extern "C"
LADSPA_Descriptor const* ladspa_descriptor(unsigned long index)
{
    if (index == 0) {
        return &(::plugin::ladspa::kDescriptor);
    }
    else {
        return NULL;
    }
}
