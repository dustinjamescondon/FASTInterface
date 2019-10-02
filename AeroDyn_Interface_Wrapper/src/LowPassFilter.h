#pragma once

class LowPassFilter
{
public:
	LowPassFilter();
	LowPassFilter(double correctingFactor);

	void SetCornerFreq(double c);

	void InitFilterVal(double);

	// Calculates, saves, and returns the estimated value at step n using observed value at step n
	double UpdateEstimate(double time, double observed);

	// Only calculates and returns the estimation at current step without updating the estimated value
	double CalcEstimation(double time, double observed) const;
	double GetCurrEstimatedValue() const;
private:

	double lastTime;
	double estimatedValue;
	double cornerFreq; // used in calculated the correcting factor
	// could save the observed value at current step, but don't need to
};

