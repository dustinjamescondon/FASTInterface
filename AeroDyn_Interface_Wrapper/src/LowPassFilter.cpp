#include "LowPassFilter.h"
#include <math.h>

LowPassFilter::LowPassFilter()
{
	estimatedValue = 0.0;
	cornerFreq = 0.0;
}

LowPassFilter::LowPassFilter(double cornerFreq) {
	this->cornerFreq = cornerFreq;
	this->estimatedValue = 0.0;
}

void LowPassFilter::SetCornerFreq(double c)
{
	cornerFreq = c;
}

void LowPassFilter::InitFilterVal(double v)
{
	estimatedValue = v;
}

// Calculates, saves, and returns the estimated value at step n using observed value at step n
double LowPassFilter::UpdateEstimate(double observed, double dt)
{
	double correctingFactor = exp(dt * cornerFreq);
	estimatedValue = (1.0 - correctingFactor) * estimatedValue + correctingFactor * observed;
	return estimatedValue;
}

// Only calculates and returns the estimation at current step without updating the estimated value
double LowPassFilter::CalcEstimation(double observed, double dt) const
{
	double correctingFactor = exp(dt * cornerFreq);
	return (1.0 - correctingFactor) * estimatedValue + correctingFactor * observed;
}

double LowPassFilter::GetCurrEstimatedValue() const
{
	return estimatedValue;
}