#ifndef WAVEFOLDER_STAGE_H
#define WAVEFOLDER_STAGE_H

class WavefolderStage{
    public:
        // Constructor
        WavefolderStage(bool antideriv, float R1, float R2, float R3);

        // Compute the wavefolder stage output
        float Process(float Vin);

        // Destructor
        ~WavefolderStage();

    private:
        float Vin_1 = 0;
        float F0_1  = 0;
        bool  antideriv;

        float      Vs;
        float      R1;
        float      R2;
        float      R3;
        float      denom  = R1 * R3 + R2 * R3 + R1 * R2;
        float      slope  = (R3 * R2) / denom;
        float      offset = (R3 * R1 * Vs) / denom;
        float      thresh;
        float      const1;
        static int signum(float x);
        float      CalcF0(float Vin);
        float      CalcFold(float Vin);
};

#endif
