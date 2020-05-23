#ifndef _LADSPAPLUGIN_HPP_
#define _LADSPAPLUGIN_HPP_

#include "Plugin.hpp"
#include "ladspa.h"

// ============================================================================

namespace plugin {
namespace ladspa {

#pragma pack(push)
#pragma pack()

// ============================================================================

enum Port {
    kPortInputMono = to_underlying(Param::kCount),
    kPortOutputLeft,
    kPortOutputRight,
    kPortCount,
};

// ============================================================================

extern LADSPA_PortDescriptor const kPortDescriptors[kPortCount];
extern LADSPA_PortRangeHint const kPortRangeHints[kPortCount];
extern LADSPA_Descriptor const kDescriptor;

// ============================================================================

LADSPA_Handle Instantiate(LADSPA_Descriptor const *descriptor, unsigned long sample_rate);
void ConnectPort(LADSPA_Handle instance, unsigned long port, LADSPA_Data *data);
void Activate(LADSPA_Handle instance);
void Run(LADSPA_Handle instance, unsigned long sample_count);
void Deactivate(LADSPA_Handle instance);
void Cleanup(LADSPA_Handle instance);

// ============================================================================

class Plugin
{
protected:
    typedef ::plugin::Plugin Wrapped;
    typedef ::plugin::Param Param;

    LADSPA_Data* ports_[kPortCount];
    LADSPA_Data cached_sliders_[to_underlying(Param::kCount)];

    unsigned long sample_rate_;
    size_t block_size_;
    Wrapped wrapped_;

protected:
    void ApplyCachedParameters()
    {
        for (unsigned i = 0; i < static_cast<unsigned>(Param::kCount); ++i) {
            LADSPA_Data current = *ports_[i];
            LADSPA_Data previous = cached_sliders_[i];
            if (current != previous) {
                cached_sliders_[i] = current;
                LADSPA_PortRangeHint const& hint = kPortRangeHints[i];
                Slider slider = LerpInv<Slider>(hint.LowerBound, hint.UpperBound, current);
                wrapped_.WriteParameter(static_cast<Param>(i), slider);
            }
        }
    }

public:
    void ConnectPort(unsigned long port, LADSPA_Data* data)
    {
        ports_[port] = data;
    }

    void Activate()
    {
        Wrapped::SampleRate sample_rate = static_cast<Wrapped::SampleRate>(sample_rate_);
        block_size_ = 1 << 12;  // FIXME
        wrapped_.Start(sample_rate, block_size_);
    }

    void Run(unsigned long sample_count)
    {
        ApplyCachedParameters();

        LADSPA_Data* inputs[YM7128B_InputChannel_Count];
        LADSPA_Data* outputs[YM7128B_OutputChannel_Count];

        for (unsigned i = 0; i < YM7128B_InputChannel_Count; ++i) {
            inputs[i] = ports_[kPortInputMono + i];
        }
        for (unsigned i = 0; i < YM7128B_OutputChannel_Count; ++i) {
            outputs[i] = ports_[kPortOutputLeft + i];
        }

        wrapped_.Process(inputs, outputs, sample_count);
    }

    void Deactivate()
    {
        wrapped_.Stop();
    }

public:
    Plugin(unsigned long sample_rate) :
        ports_(),
        cached_sliders_(),
        sample_rate_(sample_rate),
        block_size_(0)
    {
        for (unsigned i = 0; i < kPortCount; ++i) {
            ports_[i] = nullptr;
        }
        for (unsigned i = 0; i < static_cast<unsigned>(Param::kCount); ++i) {
            cached_sliders_[i] = kPortRangeHints[i].LowerBound;
        }
    }

    ~Plugin()
    {
        Deactivate();
    }
};


// ============================================================================

}  // namespace ladspa
}  // namespace plugin

#pragma pack(pop)

#endif  // _LADSPAPLUGIN_HPP_
