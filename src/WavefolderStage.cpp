#include "WavefolderStage.h"
#include <cmath>
#include <algorithm>

// Constructor
WavefolderStage::WavefolderStage(bool antideriv, float R1, float R2, float R3)
                    : antideriv(antideriv),
                      Vs(6.0f),
                      R1(R1),
                      R2(R2),
                      R3(R3) // Member initializer list
{
this->denom  = this->R1 * this->R3 + this->R2 * this->R3 + this->R1 * this->R2;
this->slope  = (this->R3 * this->R2) / this->denom;
this->offset = (this->R3 * this->R1 * this->Vs) / this->denom;

if(this->antideriv)
{
    this->thresh = this->R1 * this->Vs / this->R2;
    this->const1 = -this->R3 * this->thresh
                   * ((this->R2 * this->thresh)
                      - (2 * this->R1 * this->Vs
                         * WavefolderStage::signum(this->thresh)))
                   / (2 * this->denom);
    }
}

// Public functions
float WavefolderStage::Process(float Vin)
{
    if (this->antideriv)
    {
        float F0 = this->CalcF0(Vin);
        float eta = 0.001f;
        float VkD = 0.0f;
        float Vt1 = 0.0f;

        if(std::abs(Vin - this->Vin_1) > eta)
        {
            VkD = (F0 - this->F0_1) / (Vin - this->Vin_1);
        }
        else
        {
            Vt1 = (Vin + this->Vin_1) / 2;
            VkD = this->CalcFold(Vt1);
        }

        // Remember the Vin and F0 values for the next iteration
        this->Vin_1 = Vin;
        this->F0_1  = F0;

        return VkD;
    }
    else
    {
        return this->CalcFold(Vin);
    }
}

// Private functions
int WavefolderStage::signum(float x)
{
    return (x > 0) - (x < 0);
}

float WavefolderStage::CalcFold(float Vin)
{
    return this->signum(Vin)
           * std::max(this->slope * std::abs(Vin) - this->offset, 0.0f);
}

float WavefolderStage::CalcF0(float Vin)
{
    float sign = (float)WavefolderStage::signum(Vin);
    float VinClipped = sign * std::max((logf(std::abs(Vin) + sqrtf(powf(Vin, 2)))), this->thresh);
    return VinClipped * (this->slope * VinClipped / 2 - sign * this->offset) + this->const1;
}

WavefolderStage::~WavefolderStage()
{
}
