#include "daisysp.h"
#include "Dubby.h"
#include "wavefolderParallel.h"
#include "util/MappedValue.h"

#define NUM_AUDIO_IO 4
#define NUM_MAIN_IO  2
#define NUM_AUX_IN   2

// Namespaces...
using namespace daisy;
using namespace daisysp;

#define BUTTON_OFFSET_GAIN     Dubby::GATE_IN_1
#define BUTTON_THRESH_ATT_COMP Dubby::GATE_IN_2
#define BUTTON_RATIO_REL_COMP  Dubby::GATE_IN_3
#define BUTTON_FEEDBACK        Dubby::GATE_IN_4
#define BUTTON_FILTER_BYPASS   Dubby::GATE_IN_5

#define KNOB_OFFSET_GAIN       Dubby::CTRL_1
#define KNOB_THRESH_ATT_COMP   Dubby::CTRL_2
#define KNOB_RATIO_REL_COMP    Dubby::CTRL_3
#define KNOB_SUB_GAIN          Dubby::CTRL_4
#define JOY_FILTER_CUTOFF      Dubby::CTRL_5
#define JOY_FILTER_RESONANCE   Dubby::CTRL_6

Dubby dubby;

// DaisySP library
std::vector<Svf> svfFilter (NUM_MAIN_IO);
std::vector<daisysp::Compressor> compressor (NUM_MAIN_IO);

DaisySeed hardware;

const uint32_t sample_rate = 48000;
const bool     antideriv   = true;

// Parallel Wavefolder
std::vector<WavefolderParallel> wavefolder (NUM_MAIN_IO, WavefolderParallel(antideriv, sample_rate));
static std::vector<float> wavefolderOut(NUM_MAIN_IO, 0.0f);  // Wavefolder (L/R) output

// Mapped values
// Gain of the main inputs - multiplied with the input signal
static MappedFloatValue inGainMappedValue(0.0f, 1.0f, 0.0f, MappedFloatValue::Mapping::lin, "", 2);
// Gain of the feedback loop - multiplied with either 5V fixed output or the feedback output
static MappedFloatValue ofsFbGainMappedValue(-1.0f, 1.0f, 0.0f, MappedFloatValue::Mapping::lin, "", 2);
// Threshold of the compressor - mapped to 0 - -80dB
static MappedFloatValue thresholdMappedValue(0.0f, -80.0f, 0.0f, MappedFloatValue::Mapping::lin, "dB", 2);
// Attack of the compressor - mapped to 0.001 - 10s
static MappedFloatValue attackMappedValue(0.001f, 10.0f, 0.0f, MappedFloatValue::Mapping::log, "s", 3);
// Ratio of the compressor - mapped to 1 - 40
static MappedFloatValue ratioMappedValue(1.0f, 40.0f, 0.0f, MappedFloatValue::Mapping::lin, "", 0);
// Release of the compressor - mapped to 0.001 - 10s
static MappedFloatValue releaseMappedValue(0.001f, 10.0f, 0.0f, MappedFloatValue::Mapping::log, "s", 3);
// Cutoff frequency of the SVF filter - mapped to 20 - 20000Hz
static MappedFloatValue filterCutoffMappedValue(20.0f, 20000.0f, 0.0f, MappedFloatValue::Mapping::pow2, "Hz", 3);
// Resonance of the SVF filter - mapped to 0 - 1
static MappedFloatValue filterResonanceMappedValue(0.0f, 1.0f, 0.0f, MappedFloatValue::Mapping::lin, "", 0);
// Gain of the sub input - multiplied with the filter low pass output
static MappedFloatValue subGainMappedValue(-15.0f, 15.0f, 0.0f, MappedFloatValue::Mapping::lin, "", 2);
// Main Output of the circuit - mapped from -15 - 15V
std::vector<MappedFloatValue> outMappedValue(NUM_MAIN_IO, MappedFloatValue(-15.0f, 15.0f, 0.0f, MappedFloatValue::Mapping::lin, "V", 2));

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    // Update the mapped values depending on the button states
    // Button 1 (BUTTON_OFFSET_GAIN]) controls the mode of Knob 1 (analogInputs[CTRL_1]) (offset or gain of main inputs)
    if(dubby.gateInputs[BUTTON_OFFSET_GAIN].State())
    {
        inGainMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_OFFSET_GAIN)));
    }
    else
    {
        ofsFbGainMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_OFFSET_GAIN)));
    }

    // Button 2 (BUTTON_THRESH_ATT_COMP) controls the mode of Knob 2 (analogInputs[CTRL_2]) (threshold or attack of compressor)
    if(dubby.gateInputs[BUTTON_THRESH_ATT_COMP].State())
    {
        thresholdMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_THRESH_ATT_COMP)));
    }
    else
    {
        attackMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_THRESH_ATT_COMP)));
    }

    // Button 3 (BUTTON_RATIO_REL_COMP) controls the mode of Knob 3 (analogInputs[CTRL_2]) (ratio or release of compressor)
    if(dubby.gateInputs[BUTTON_RATIO_REL_COMP].State())
    {
        ratioMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_RATIO_REL_COMP)));
    }
    else
    {
        releaseMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_RATIO_REL_COMP)));
    }

    // Update Compressor parameters
    for(int i = 0; i < NUM_MAIN_IO; i++)
    {
        compressor[i].SetThreshold(thresholdMappedValue.Get());
        compressor[i].SetRatio(ratioMappedValue.Get());
        compressor[i].SetAttack(attackMappedValue.Get());
        compressor[i].SetRelease(releaseMappedValue.Get());
    }

    // Knob 4 (KNOB_SUB_GAIN) controls the gain of the sub input
    subGainMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(KNOB_SUB_GAIN)));
    // The Joystick X axis (JOY_FILTER_CUTOFF controls the filter cutoff frequency)
    filterCutoffMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(JOY_FILTER_CUTOFF)));
    // The Joystick Y axis (JOY_FILTER_RESONANCE controls the filter resonance)
    filterResonanceMappedValue.SetFrom0to1(dubby.GetKnobValue(static_cast<Dubby::Ctrl>(JOY_FILTER_RESONANCE)));
    // Update SVF filter parameters
    for(int i = 0; i < NUM_MAIN_IO; i++)
    {
        svfFilter[i].SetFreq(filterCutoffMappedValue.Get());
        svfFilter[i].SetRes(filterResonanceMappedValue.Get());
    }

    double sumSquared[4] = {0.0f};

    for(size_t i = 0; i < size; i++)
    {
        for(int j = 0; j < NUM_MAIN_IO; j++)
        {
            // Button 4 (gateInputs[GATE_IN_4]) switches the feedback loop on or off (if off, attenuvert 5V with the feedback gain knob)
            float feedbackSignal;
            if (dubby.gateInputs[BUTTON_FEEDBACK].State())
            {
                feedbackSignal = wavefolderOut[j];
            }
            else
            {
                feedbackSignal = 5.0f;
            }

            // Compressor
            float compressorIn = in[j][i] * inGainMappedValue.Get() + ofsFbGainMappedValue.Get() * feedbackSignal;
            float compressorOut = compressor[j].Process(compressorIn, in[j + NUM_MAIN_IO][i]);

            // The Joystick Button (gateInputs[GATE_IN_5]) switches the filter bypass on or off
            float filterNotchOut;
            float filterLowOut;
            if(dubby.gateInputs[BUTTON_FILTER_BYPASS].State())
            {
                filterNotchOut = compressorOut;
                filterLowOut   = 0.0f;
            }
            else
            {
                svfFilter[j].Process(compressorOut);
                filterNotchOut = svfFilter[j].Notch();
                filterLowOut   = svfFilter[j].Low();
            }

            wavefolderOut[j] = wavefolder[j].Process(filterNotchOut);
            outMappedValue[j].Set(wavefolderOut[j] + subGainMappedValue.Get() * filterLowOut);

            // Normalize the output
            out[j][i] = outMappedValue[j].GetAs0to1();

            // Update current levels
            sumSquared[j] += out[j][i] * out[j][i];
        }

        // Update the scope buffer
        dubby.scope_buffer[i] = (out[0][i] + out[1][i]) * .5f;
    }

    for(int j = 0; j < 4; j++)
        dubby.currentLevels[j] = sqrt(sumSquared[j] / AUDIO_BLOCK_SIZE);
}

int main(void)
{
    dubby.seed.Init();
    dubby.seed.StartLog(true);

    dubby.Init();

    dubby.seed.PrintLine("Dubby Playground");

    dubby.seed.SetAudioBlockSize(
        AUDIO_BLOCK_SIZE); // number of samples handled per callback
    dubby.seed.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    dubby.ProcessAllControls();

    dubby.DrawLogo();
    System::Delay(1000);
    dubby.seed.StartAudio(AudioCallback);
    dubby.UpdateMenu(0, false);

    // Compressor init
    compressor[0].Init(sample_rate);

    // SVF filter init
    svfFilter[0].Init(sample_rate);
    svfFilter[0].SetFreq(1000);
    svfFilter[0].SetDrive(1);
    svfFilter[0].SetRes(1);
    svfFilter[1] = svfFilter[0];

    while(1)
    {
        // Read the knobs and buttons
        dubby.ProcessAllControls();
        dubby.UpdateDisplay();
    }
}
