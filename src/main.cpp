#include <daisy_seed.h>
#include "stm32h7xx.h"
#include "arm_math.h"

#include "daisysp.h"
#include "Dubby.h"

// #ifndef PI
// #define PI 3.14159265358979f
// #endif

// #include <q/pitch/pitch_detector.hpp>
// #include <q/support/notes.hpp>
// #include <q/fx/biquad.hpp>
// #include <q/synth/sin.hpp>

// Namespaces...
using namespace daisy;
using namespace daisysp;

Dubby dubby;

// DaisySP library
Svf SvfFilter;

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

static int signum(float x)
{
    return (x > 0) - (x < 0);
}

float WavefoldStage(float Vs, float Vin, float R1, float R2, float R3)
{
    float denom  = R1 * R3 + R2 * R3 + R1 * R2;
    float slope  = (R3 * R2) / denom;
    float offset = (R3 * R1 * Vs) / denom;
    return signum(Vin) * std::max(slope * std::abs(Vin) - offset, 0.0f);
}

float CalcF0(float Vin, float thresh, float slope, float offset, float const1)
{
    float sign       = signum(Vin);
    float VinClipped = sign * std::max((logf(std::abs(Vin) + sqrtf(powf(Vin, 2)))), thresh);
    return VinClipped * (slope * VinClipped / 2 - sign * offset) + const1;
}

float WavefoldStageAntiderived(float Vs, float Vin, float R1, float R2, float R3)
{
    static float Vin_1 = 0;
    static float F0_1 = 0;

    float denom  = R1 * R3 + R2 * R3 + R1 * R2;
    float slope  = (R3 * R2) / denom;
    float offset = (R3 * R1 * Vs) / denom;
    float thresh = R1 * Vs / R2;
    float const1 = -R3 * thresh * (R2 * thresh - 2 * R1 * Vs * signum(thresh)) / (2 * denom);

    float F0 = CalcF0(Vin, thresh, slope, offset, const1);

    float eta = 0.001f;
    float VkD = 0.0f;
    float Vt1 = 0.0f;

    if(std::abs(Vin - Vin_1) > eta)
    {
        VkD = (F0 - F0_1) / (Vin - Vin_1);
    }
    else
    {
        Vt1 = (Vin + Vin_1) / 2;
        VkD = WavefoldStage(Vs, Vt1, R1, R2, R3);
    }

    // Remember the Vin and F0 values for the next iteration
    Vin_1  = Vin;
    F0_1   = F0;

    return VkD;
}

float SummingStage1(float Vin, float V4, float V5, float RF1, float* R3)
{
    return -RF1 * (V4 / *(R3 + 0) + V5 / *(R3 + 1) + Vin / *(R3 + 2));
}

float SummingStage2(float Vin, float V1, float V2, float V3, float RF2, float *R3, float R7)
{
    return -RF2 * (V1 / *(R3 + 0) + V2 / *(R3 + 1) + V3 / *(R3 + 2) + Vin / R7);
}

float FilteringStage(float Vin, float sampleRate, float R, float C)
{
    float fc      = 1 / (2 * PI * R * C);
    float divisor = 2 * PI * fc / sampleRate;
    float b[2]    = {1 - 2 / (2 + divisor), 1 - 2 / (2 + divisor)};
    float a[2]    = {1, 1 - 4 / (2 + divisor)};

    static float x[2] = {Vin, 0.0f};
    static float y[2] = {0.0f, 0.0f};

    x[1]      = x[0];
    x[0]      = Vin;

    float out = (1.0f / a[0]) * (x[0] * b[0] + x[1] * b[1] - y[1] * a[1]);

    y[1] = y[0];
    y[0] = out;

    return out;
}

float WavefolderParallel(float Vin, float sampleRate, bool antideriv)
{
    std::vector<float> R1 = {
                                10.0f * 1e3,
                                49.9f * 1e3,
                                91.0f * 1e3,
                                30.0f * 1e3,
                                68.0f * 1e3
                            };
    std::vector<float> R2(std::size(R1), 100.0f * 1e3);
    std::vector<float> R3 = {
                                100.0f * 1e3,
                                43.2f * 1e3,
                                56.0f * 1e3,
                                68.0f * 1e3,
                                33.0f * 1e3,
                                240.0f * 1e3
                            } ;
    float C   = 100.0f * 1e-12;
    float R7  = 24.9f * 1e3;
    float RF1 = 24.9f * 1e3;
    float RF2 = 1.2f * 1e6;
    float Vs  = 6.0f;

    std::vector<float> Fold_Stage_Vouts(std::size(R1), 0.0f);
    for(unsigned int k = 0; k < std::size(R1); k++)
    {
        if(antideriv)
        {
            Fold_Stage_Vouts[k] = WavefoldStageAntiderived(Vs, Vin, R1[k], R2[k], R3[k]);
        }
        else
        {
            Fold_Stage_Vouts[k] = WavefoldStage(Vs, Vin, R1[k], R2[k], R3[k]);
        }
    }
    float Summing_stage1_Vout = 0.0f;
    Summing_stage1_Vout = SummingStage1(Vin,
                                        Fold_Stage_Vouts[3],
                                        Fold_Stage_Vouts[4],
                                        RF1,
                                        &R3[3]);
    float Summing_stage2_Vout = 0.0f;

    Summing_stage2_Vout = SummingStage2(Summing_stage1_Vout,
                                        Fold_Stage_Vouts[0],
                                        Fold_Stage_Vouts[1],
                                        Fold_Stage_Vouts[2],
                                        RF2,
                                        &R3[0],
                                        R7);

    float Filtering_stage_Vout = 0.0f;
    Filtering_stage_Vout = FilteringStage(Summing_stage2_Vout, sampleRate, RF2, C);
    return Filtering_stage_Vout;
    // return Summing_stage2_Vout;
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    double sumSquared[4] = {0.0f};

    float offset = 0.0f;
    for(size_t i = 0; i < size; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // out[j][i] = dubby.GetKnobValue(static_cast<Dubby::Ctrl>(j)) * in[j][i];
            float gain = dubby.GetKnobValue(static_cast<Dubby::Ctrl>(j));
            out[j][i]  = WavefolderParallel(offset + gain * in[j][i], 48000, true);

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

    // SVF filter init
    SvfFilter.Init(48000);
    SvfFilter.SetFreq(1000);
    SvfFilter.SetDrive(1);
    SvfFilter.SetRes(1);

    while(1)
    {
        dubby.ProcessAllControls();
        dubby.UpdateDisplay();
    }
}
