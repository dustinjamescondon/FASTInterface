#include "LowPassFilter.h"

LowPassFilter::LowPassFilter()
{
	correctingFactor = 0.0;
	estimatedValue = 0.0;
}

LowPassFilter::LowPassFilter(double correctingFactor) {
	this->correctingFactor = correctingFactor;
	this->estimatedValue = 0.0;
}

void LowPassFilter::SetCorrectingFactor(double c)
{
	correctingFactor = c;
}

// Calculates, saves, and returns the estimated value at step n using observed value at step n
 double LowPassFilter::UpdateEstimate(double observed) {
	estimatedValue = (1.0 - correctingFactor) * estimatedValue + correctingFactor * observed;
	return estimatedValue;
}

// Only calculates and returns the estimation at current step without updating the estimated value
 double LowPassFilter::CalcEstimation(double observed) const {
	return (1.0 - correctingFactor) * estimatedValue + correctingFactor * observed;
}

 double LowPassFilter::GetCurrEstimatedValue() const
{
	return estimatedValue;
}