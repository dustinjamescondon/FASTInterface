#pragma once

#include <vector>

class InflowState {
public:
	InflowState();
	~InflowState();

	void allocate(int nBlades);
	void updateInflow(const std::vector<double>& inflows, double time);
	vector<double> interpolateInflows(double time);

private:
	double* newInflows;
	double* oldInflows;

	double oldInflowTime;
	double newInflowTime;
};
