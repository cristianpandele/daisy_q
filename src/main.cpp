#include <daisy_seed.h>
#include "daisysp.h"
#include "Dubby.h"

// #include <q/pitch/pitch_detector.hpp>
// #include <q/support/notes.hpp>
// #include <q/fx/biquad.hpp>
// #include <q/synth/sin.hpp>

// Namespaces...
using namespace daisy;
using namespace daisysp;

Dubby dubby;

// Q library
// namespace q = cycfi::q;
// using namespace cycfi::q::literals;

DaisySeed hardware;

uint32_t sample_rate = 48000;

// // The dectected frequencies
// q::frequency detected_f0 = q::frequency(0.0f);

// // The frequency detection bounds;
// q::frequency lowest_frequency  = q::notes::C[2];
// q::frequency highest_frequency = q::notes::C[5];

// // The pitch detector pre-processor
// q::pd_preprocessor::config preprocessor_config;
// q::pd_preprocessor         preprocessor{preprocessor_config,
//                                 lowest_frequency,
//                                 highest_frequency,
//                                 sample_rate};

// // The pitch detector
// q::pitch_detector pd{lowest_frequency, highest_frequency, sample_rate};

// // A phase accumulator for our synthesized output
// q::phase phase_accumulator = q::phase();
// q::phase f0_phase          = q::phase(detected_f0, sample_rate);

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    double sumSquared[4] = {0.0f};

    for(size_t i = 0; i < size; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            out[j][i] = dubby.GetKnobValue(static_cast<Dubby::Ctrl>(j)) * in[j][i];

            float sample = out[j][i];
            sumSquared[j] += sample * sample;
        }

        dubby.scope_buffer[i] = (out[0][i] + out[1][i]) * .5f;
    }

    for(int j = 0; j < 4; j++)
        dubby.currentLevels[j] = sqrt(sumSquared[j] / AUDIO_BLOCK_SIZE);
}

int main(void)
{
    dubby.seed.Init();
    // dubby.InitAudio();
    // dubby.seed.StartLog(true);

    dubby.Init();

    dubby.seed.SetAudioBlockSize(
        AUDIO_BLOCK_SIZE); // number of samples handled per callback
    dubby.seed.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    dubby.ProcessAllControls();

    dubby.DrawLogo();
    System::Delay(1000);
    dubby.seed.StartAudio(AudioCallback);
    dubby.UpdateMenu(0, false);

    while(1)
    {
        dubby.ProcessAllControls();
        dubby.UpdateDisplay();
    }
}
