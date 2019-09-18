#pragma once
// Author: Dustin Condon 
// File  : CSVSpecFunction.h

// "Comma-Seperated-Variable Specified Function"
// Defines a function F(x), where the domain x is the first column of the CSV table, and the range
// of F(x) is specified by the second column. The function is assumed to be contiguous, so any parts 
// in between the specified values is linearly interpolated
//
// The table is expected to have only 2 columns; 

#include <vector>
#include <fstream>
#include "ControllerExceptions.h"

class CSVSpecFunction
{
public:
	CSVSpecFunction(std::ifstream&);
	CSVSpecFunction(const char* filename);
	CSVSpecFunction() noexcept;

	// Loads the specified CSV file to define the function
	void loadCSVFile(const char* filename);

	// Reads in the data a from an open file stream to define the function
	void readCSVFile(std::ifstream& fin);

	// Returns the output of the function based upon the input x.
	double F(double x) const;

	double getMaxSpecX() const; // returns the largest x that was specified in the CSV table
	double getMinSpecX() const; // returns the smallest x that was specified in the CSV table

private:

	struct Pair {
		double x, Fofx;
	};

	void validateInput();

	void calcSamplePeriod();
	void calcMinSpecDomain();
	void calcMaxSpecDomain();
	void calcSpecRangeHead();
	void calcSpecRangeTail();

	double interpolate(const Pair&, const Pair&, double x) const;

	double deltaX;              // the sample period
	double minSpecDomain;       // the minimum x value specified in the CSV file
	double maxSpecDomain;       // the maximum x value specified in the CSV file
	double specRangeHead;       // the value of Fofx at minDomain
	double specRangeTail;       // the value of Fofx at maxDomain
	std::vector<Pair> pairs;    // holds all the x/Fofx pairs
};