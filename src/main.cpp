#include "daisysp.h"
#include "Dubby.h"
#include "wavefolderParallel.h"

#define NUM_AUDIO_IO 4
#define NUM_MAIN_IO  2
#define NUM_AUX_IN   2

// #include <q/pitch/pitch_detector.hpp>
// #include <q/support/notes.hpp>
// #include <q/fx/biquad.hpp>
// #include <q/synth/sin.hpp>

// Namespaces...
using namespace daisy;
using namespace daisysp;

Dubby dubby;

// DaisySP library
std::vector<Svf> svfFilter (NUM_MAIN_IO);
std::vector<daisysp::Compressor> compressor (NUM_MAIN_IO);

// Q library
// namespace q = cycfi::q;
// using namespace cycfi::q::literals;

DaisySeed hardware;

uint32_t sample_rate = 48000;
bool     antideriv   = true;

// Parallel Wavefolder
// WavefolderParallel wavefolder = WavefolderParallel(antideriv, sample_rate);
std::vector<WavefolderParallel> wavefolder (NUM_AUDIO_IO, WavefolderParallel(antideriv, sample_rate));


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
    // double sumSquared[NUM_AUDIO_IO] = {0.0f};

    float offset = 0.0f;
    for(size_t i = 0; i < size; i++)
    {
        for(int j = 0; j < NUM_AUDIO_IO; j++)
        {
            // out[j][i] = dubby.GetKnobValue(static_cast<Dubby::Ctrl>(j)) * in[j][i];
            float gain = dubby.GetKnobValue(static_cast<Dubby::Ctrl>(j));
            if (j < NUM_MAIN_IO)
            {
                out[j][i] = wavefolder[j].Process(offset + gain * in[j][i]);
            }

            // Button 1 (gateInputs[GATE_IN_1]) controls the mode of Knob 1 (analogInputs[CTRL_1]) (offset or gain of inputs 1/2)
            // Button 2 (gateInputs[GATE_IN_2]) controls the mode of Knob 2 (ana    Inputs[CTRL_2]) (threshold or attack of compressor)
            // Button 3 (gateInputs[GATE_IN_3]) controls the mode of Knob 3 (analogInputs[CTRL_2]) (ration or release of compressor)
            // Button 4 (gateInputs[GATE_IN_4]) switches the feedback loop (analogInputs[CTRL_4]) (offset or gain of inputs 1/2)
            // The Joystick Button (gateInputs[GATE_IN_5]) switches the filter bypass on or off
            // The Joystick X axis (analogInputs[CTRL_5] controls the filter cutoff frequency)
            // The Joystick Y axis (analogInputs[CTRL_6] controls the filter resonance frequency)

            // float sample = out[j][i];
            // sumSquared[j] += sample * sample;
        }

        // dubby.scope_buffer[i] = (out[0][i] + out[1][i]) * .5f;
    }

    // for(int j = 0; j < 4; j++)
    //     dubby.currentLevels[j] = sqrt(sumSquared[j] / AUDIO_BLOCK_SIZE);
}

int main(void)
{
    dubby.seed.Init();
    // dubby.InitAudio();
    // dubby.seed.StartLog(true);

    dubby.Init();

    // Compressor init
    compressor[0].Init(sample_rate);

    // SVF filter init
    svfFilter[0].Init(sample_rate);
    svfFilter[0].SetFreq(1000);
    svfFilter[0].SetDrive(1);
    svfFilter[0].SetRes(1);
    svfFilter[1] = svfFilter[0];


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
