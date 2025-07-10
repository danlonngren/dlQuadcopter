#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

class Fuzzy {
  private:
    float m_integralError;
    float m_w[5];
    float m_fuzzyOutput;
    float m_eLast;
    float m_outputMax;
    float m_errorLimit, m_dErrorLimit, m_integralLimit;
    float output_gain;
    float rate_gain;
    
    struct Rule {
        std::function<float()> condition; // Activation function
        float weight;
        float output;
    };
    std::vector<Rule> rules;

  public:
    Fuzzy();
    Fuzzy(float e_limit, float de_limit, float re_limit, float output_gain, float out_max);

    float Controller(float rollInput, float setpoint, bool Level);
    void setLimits(float e_limit, float de_limit, float re_limit, float output_gain, float out_max);
    void setOutputMax(float out_max);
    void reset();

    // Member functions for fuzzy logic operations
    float normalize(float x, float limit) const;

    float gaussianMF(float  x, float sigma);
    float linearPositiveMF(float x, float maxValue);
    float linearNegativeMF(float x, float maxValue);

    float linearSignedMF(float x, float limit);

    // TODO: Fix
    float positiveNL(float x);
    float negativeNL(float x);

};

#endif
