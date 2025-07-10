#include "fuzzyController.h"

#include <math.h>
#include <functional>
#include <vector>
#include <memory>

template<typename T>
T clamp(T val, T minVal, T maxVal) {
    if (val < minVal) return minVal;
    else if (val > maxVal) return maxVal;
    else return val;
}

Fuzzy::Fuzzy() :
    m_errorLimit(0), m_dErrorLimit(0), m_integralLimit(0), 
    output_gain(0), m_outputMax(0), m_fuzzyOutput(0.0f) {
    m_integralError = 0.0f;
    m_eLast = 0.0f;
}

Fuzzy::Fuzzy(float e_limit, float de_limit, float re_limit, float output_gain, float m_outputMax) :
    m_errorLimit(e_limit), m_dErrorLimit(de_limit), m_integralLimit(re_limit), 
    output_gain(output_gain), m_outputMax(m_outputMax) {
    m_integralError = 0.0f;
    m_eLast = 0.0f;
    m_fuzzyOutput = 0.0f;
}

void Fuzzy::setLimits(float e_limit, float de_limit, float re_limit, float output_gain, float m_outputMax) {
	m_errorLimit = e_limit;
	m_dErrorLimit = de_limit;
	m_integralLimit = re_limit;
	this->output_gain = output_gain;
	this->m_outputMax = m_outputMax;
}

void Fuzzy::setOutputMax(float m_outputMax) {
	this->m_outputMax = m_outputMax;
}

float Fuzzy::gaussianMF(float  x, float sigma) {
    if (sigma <= 0) return 0.0f; // Avoid division by zero
    return clamp(expf((-(x * x)) / (2.0f * sigma * sigma)), 0.0f, 1.0f);
}

float Fuzzy::linearPositiveMF(float x, float maxValue) {
    return clamp((x + maxValue) / (2 * maxValue), 0.0f, 1.0f);
}

float Fuzzy::linearNegativeMF(float x, float maxValue) {
    return clamp((-x + maxValue) / (2 * maxValue), 0.0f, 1.0f);
}

float Fuzzy::positiveNL(float x) {
    return (((x * x * x + x) / 2020.0) + 0.5);
}

float Fuzzy::negativeNL(float x) {
    return (((-x * x * x - x) / 2020.0) + 0.5);
}

float Fuzzy::linearSignedMF(float x, float limit) {
    if (limit <= 0.0f) return 0.0f;
    float norm = x / limit;
    return clamp(norm, -1.0f, 1.0f);
}

float Fuzzy::normalize(float x, float limit) const {
    return clamp(x / std::abs(limit), -1.0f, 1.0f);
}

float Fuzzy::Controller(float roll_input, float setpoint, bool level) {
	float singletonOutputValue = 1.0f;
    
    // --- Compute error terms ---
	float error = (setpoint - roll_input);  // Error
	float dError = (error - m_eLast);           // Derivative error
    m_eLast = error;

    // Integral error with anti-windup
    m_integralError = clamp(m_integralError + error, -m_integralLimit, m_integralLimit);

#if 1
    float ne = normalize(error, m_errorLimit);
    float nde = normalize(dError, m_dErrorLimit);
    float nre = normalize(m_integralError, m_integralLimit);

    auto negLMF = std::make_shared<LinearNegativeMF>();
    auto posLMF = std::make_shared<LinearPositiveMF>();    
    auto gauMF = std::make_shared<GaussianMF>();

    rules = {
        // P- and P+ rules
        { [=]() { return negLMF->evaluate(ne) * negLMF->evaluate(nde); }, m_w[1], -1.0f },
        { [=]() { return posLMF->evaluate(ne) * posLMF->evaluate(nde); }, m_w[1],  1.0f },
        // D+ and D- rules
        { [=]() { return negLMF->evaluate(ne) * posLMF->evaluate(nde); }, m_w[2],  1.0f },
        { [=]() { return posLMF->evaluate(ne) * negLMF->evaluate(nde); }, m_w[2], -1.0f },
        // I- and I+ rules
        { [=]() { return negLMF->evaluate(ne) * negLMF->evaluate(nre); }, m_w[3], -1.0f },
        { [=]() { return posLMF->evaluate(ne) * posLMF->evaluate(nre); }, m_w[3],  1.0f },
        // Gaussian rules for reducing overshoot
        { [=]() { return  gauMF->evaluate(ne) * posLMF->evaluate(nde); }, m_w[4],  1.0f },
        { [=]() { return  gauMF->evaluate(ne) * negLMF->evaluate(nde); }, m_w[4], -1.0f },
    };

    float numerator = 0.0f;
    float denominator = 0.0f;

    for (const auto& rule : rules) {
        float a = rule.condition();
        numerator += a * rule.weight * rule.output;
        denominator += a;
    }

    float output = (denominator != 0.0f) ? numerator / denominator : 0.0f;
    output *= output_gain;
    output = clamp(output, -m_outputMax, m_outputMax);

#else

    // --- Fuzzify inputs ---
	float posErrorMF = linearPositiveMF(error, m_errorLimit); // linear posetive function (centre point 0.5 with output from 0 to 1)
	float negErrorMF = linearNegativeMF(error, m_errorLimit);
    
	float posDEMF = linearPositiveMF(dError, m_dErrorLimit);
	float negDEMF = linearNegativeMF(dError, m_dErrorLimit);

	float posIMF = linearPositiveMF(m_integralError, m_integralLimit);
	float negIMF = linearNegativeMF(m_integralError, m_integralLimit);
    
	float gauErrorMF = gaussianMF(error, m_errorLimit); // Centered 0

    // --- Rule base ---
    float rule[8] = {0.0f};
	rule[0] = negErrorMF * negDEMF; // P- rule 
	rule[1] = posErrorMF * posDEMF; // P+ rule
	rule[2] = negErrorMF * posDEMF; // D+ rule
	rule[3] = posErrorMF * negDEMF; // D- rule
	rule[4] = negErrorMF * negIMF;  // I- rule
	rule[5] = posErrorMF * posIMF;  // I+ rule
	rule[6] = gauErrorMF * posDEMF; // r6 - reduce overshoot, emphasize D+
	rule[7] = gauErrorMF * negDEMF; // r7 - reduce overshoot, emphasize D-

    // --- Ser mode specific rule weights ---
    if (level) 	{
		m_w[0] = 1.0;
		m_w[1] = 0.35;
		m_w[2] = 1.0;
		m_w[3] = 0.7;
	} else {
		m_w[0] = 1.3;
		m_w[1] = 0.15;
		m_w[2] = 1.0;
		m_w[3] = 0;
	}

	// --- Rule outputs ---
    float ruleOutput[8] = {0.0f};
	ruleOutput[0] = rule[0] * m_w[0] * (-singletonOutputValue);
	ruleOutput[1] = rule[1] * m_w[0] * ( singletonOutputValue);
	ruleOutput[2] = rule[2] * m_w[1] * ( singletonOutputValue);
	ruleOutput[3] = rule[3] * m_w[1] * (-singletonOutputValue);
	ruleOutput[4] = rule[4] * m_w[2] * (-singletonOutputValue);
	ruleOutput[5] = rule[5] * m_w[2] * ( singletonOutputValue);
	ruleOutput[6] = rule[6] * m_w[3] * ( singletonOutputValue);
	ruleOutput[7] = rule[7] * m_w[3] * (-singletonOutputValue);

    // --- Centroidal Defuzzifier ---
    float sumA = 0.0f;
    float sumU = 0.0f;
    for (int i = 0; i < 8; ++i) {
        sumA += rule[i];
        sumU += ruleOutput[i];
    }
	m_fuzzyOutput = (sumA > 0.001f) ? (sumU / sumA) : 0.0f; // Avoid division by zero

    // --- Apply rate gain ---
	m_fuzzyOutput *= output_gain;

    // --- Output limiter ---
	m_fuzzyOutput = clamp(m_fuzzyOutput, -m_outputMax, m_outputMax);

#endif

	return m_fuzzyOutput;
}

void Fuzzy::reset() {
	m_integralError = 0;
	m_fuzzyOutput = 0;
}

// Membership functions interface
class MembershipFunction {
public:
    virtual float evaluate(float x) const = 0;
    virtual ~MembershipFunction() = default;
};

class GaussianMF : public MembershipFunction {
public:
    float evaluate(float x) const override {
        return expf(-(x * x) / 2.0f); // sigma = 1.0 assumed
    }
};

class LinearPositiveMF : public MembershipFunction {
public:
    float evaluate(float x) const override {
        return clamp((x + 1.0f) / 2.0f, 0.0f, 1.0f);
    }
};

class LinearNegativeMF : public MembershipFunction {
public:
    float evaluate(float x) const override {
        return clamp((1.0f - x) / 2.0f, 0.0f, 1.0f);
    }
};