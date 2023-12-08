#ifndef WAVEFOLDER_PARALLEL_H
#define WAVEFOLDER_PARALLEL_H
#include <vector>
#include "WavefolderStage.h"

class WavefolderParallel
{
    public:
        // Constructor
        WavefolderParallel(bool antideriv, int sampleRate);

        // Compute the WavefolderParallel output
        float Process(float Vin);

        // Destructor
        ~WavefolderParallel();

    private:
        static const int NUM_STAGES = 5;
        float            antideriv;
        int              sampleRate;

        const std::vector<float> R1  = {10.0f * 1e3,
                                        49.9f * 1e3,
                                        91.0f * 1e3,
                                        30.0f * 1e3,
                                        68.0f * 1e3};
        const std::vector<float> R2  = {100.0f * 1e3,
                                        100.0f * 1e3,
                                        100.0f * 1e3,
                                        100.0f * 1e3,
                                        100.0f * 1e3};
        const std::vector<float> R3  = {100.0f * 1e3,
                                        43.2f * 1e3,
                                        56.0f * 1e3,
                                        68.0f * 1e3,
                                        33.0f * 1e3,
                                        240.0f * 1e3};

        const float C   = 100.0f * 1e-12;
        const float R7  = 24.9f * 1e3;
        const float RF1 = 24.9f * 1e3;
        const float RF2 = 1.2f * 1e6;
        const float Vs  = 6.0f;

        std::vector<WavefolderStage> stage;
        float                        SummingStage(float                     gain,
                                                  const std::vector<float>& voltages,
                                                  const std::vector<float>& resistances);
        float                        FilteringStage(float Vin);
};
#endif // !WAVEFOLDER_PARALLEL_H