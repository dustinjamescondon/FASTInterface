#pragma once

class LowPassFilter
{
public:
	LowPassFilter();
	LowPassFilter(double correctingFactor);

	void SetCorrectingFactor(double c);

	// Calculates, saves, and returns the estimated value at step n using observed value at step n
	double UpdateEstimate(double observed);

	// Only calculates and returns the estimation at current step without updating the estimated value
	double CalcEstimation(double observed) const;
	double GetCurrEstimatedValue() const;
private:

	double estimatedValue;
	double correctingFactor;
	// could save the observed value at current step, but don't need to
};

