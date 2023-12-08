#include "stm32h7xx.h"
#include "arm_math.h"

#include "WavefolderParallel.h"

// Constructor
WavefolderParallel::WavefolderParallel(bool antideriv, int sampleRate)
{
    this->antideriv = antideriv;
    this->sampleRate = sampleRate;

    for(int i = 0; i < this->NUM_STAGES; i++)
    {
        stage[i] = WavefolderStage(this->antideriv, this->R1[i], this->R2[i], this->R3[i]);
    }
}

// Public functions
float WavefolderParallel::Process(float Vin)
{
    std::vector<float> foldStageVout(this->NUM_STAGES, 0.0f);

    std::vector<float> voltages1    = {this->stage[3].Process(Vin),
                                       this->stage[4].Process(Vin),
                                       Vin};
    std::vector<float> resistances1 = {this->R3[3],
                                       this->R3[4],
                                       this->R3[5]};
    float summingStage1Vout         = this->SummingStage(this->RF1, voltages1, resistances1);

    std::vector<float> voltages2    = {this->stage[0].Process(Vin),
                                            this->stage[1].Process(Vin),
                                            this->stage[2].Process(Vin),
                                            summingStage1Vout};
    std::vector<float> resistances2 = {this->R3[01],
                                            this->R3[1],
                                            this->R3[2],
                                            this->R7,
                                            this->R3[5]};

    float summingStage2Vout         = this->SummingStage(this->RF2, voltages2, resistances2);

    float filterStageVout           = FilteringStage(summingStage2Vout);
    return filterStageVout;
    // return summingStage2Vout;
}

WavefolderParallel::~WavefolderParallel()
{
}

// Private functions
float WavefolderParallel::SummingStage(float gain,
                               const std::vector<float>& voltages,
                               const std::vector<float>& resistances)
{
    float sum = 0.0f;

    for(size_t i = 0; i < voltages.size(); i++)
    {
        sum += voltages[i] / resistances[i];
    }
    return -gain * sum;
}

float WavefolderParallel::FilteringStage(float Vin)
{
    float fc      = 1 / (2 * PI * this->RF2 * this->C);
    float divisor = 2 * PI * fc / this->sampleRate;
    float b[2]    = {1 - 2 / (2 + divisor), 1 - 2 / (2 + divisor)};
    float a[2]    = {1, 1 - 4 / (2 + divisor)};

    static float x[2] = {Vin, 0.0f};
    static float y[2] = {0.0f, 0.0f};

    x[1] = x[0];
    x[0] = Vin;

    float out = (1.0f / a[0]) * (x[0] * b[0] + x[1] * b[1] - y[1] * a[1]);

    y[1] = y[0];
    y[0] = out;

    return out;
}