#include "InflowState.h"

InflowState::InflowState() {
	oldInflows = 0;
	newInflows = 0;
}

void InflowState::allocate(int totalNodes) {
	oldInflows = new double[totalNodes];
}

void updateInflow(const std::vector<double>& inflows, double time) {
	double* hold = oldInflows;
	oldInflows = newInflows;
	newInflows = hold;

	for(int i = 0; i < )

}