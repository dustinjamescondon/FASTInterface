#include "CSVSpecFunction.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <assert.h>
#include <math.h>
#include "FASTTurbineExceptions.h"

CSVSpecFunction::CSVSpecFunction(const char* filename)
{
  LoadCSVFile(filename);
}

CSVSpecFunction::CSVSpecFunction(std::ifstream& fin)
{
  ReadCSVFile(fin);
}

CSVSpecFunction::CSVSpecFunction() noexcept
{
  deltaX = minSpecDomain = maxSpecDomain = specRangeHead = specRangeTail = 0;
}

// Assumes that the first line of the CSV just contains labels, so it ignores it.
void CSVSpecFunction::LoadCSVFile(const char* filename) {
  std::ifstream fin(filename);

  // check if file was found and opened
  if (fin.is_open()) {
	
    // ignore the  first line because it should contain the column labels)
    std::string line;
    std::getline(fin, line, '\n');

    ReadCSVFile(fin);

    fin.close();
  }
  else {
    // error
    std::string errorMsg = std::string("Couldn't open CSV file: ") + std::string(filename);
    throw FileNotFoundException(errorMsg.c_str());
  }
}

// Does not assume the first available line is the comment row, so the first row is NOT ignored
void CSVSpecFunction::ReadCSVFile(std::ifstream& fin)
{
  // holds a line from the file
  std::string line;

  while (std::getline(fin, line, '\n'))
    {
      unsigned int i = 0;
      double pair[2]; // used to hold the first two tokens of each row

      // parse the line, saving only the first two entries
      std::stringstream linestream(line);
      std::string token;
      while (std::getline(linestream, token, ',')) {
	// if we're still reading in the first two colums,
	//    save them 
	if (i < 2) {
	  pair[i++] = stod(token);
	}
	// otherwise, stop parsing this line, and move on to the next
	else {
	  break;
	}
      }

      // save the first and second token
      Pair p;
      p.x = pair[0];
      p.Fofx = pair[1];
      pairs.push_back(p);
    }

  // Make sure the input file is correctly formatted (throws an exception if not)
  ValidateInput();

  // determine the sampling period
  CalcSamplePeriod();

  CalcMinSpecDomain();
  CalcMaxSpecDomain();
  CalcSpecRangeHead();
  CalcSpecRangeTail();
}

// TODO add some more error checking
// Checks to see if the content loaded from the input file is valid. Throws an exception
// if it isn't.
void CSVSpecFunction::ValidateInput() {

  if (pairs.size() <= 1) {
    std::string errMsg("The table has less than two sample points.");
    throw FileContentsException(errMsg.c_str());
  }
}


// just takes the first two entries and takes the absolute value of their difference
void CSVSpecFunction::CalcSamplePeriod()
{
  deltaX = fabs(pairs[0].x - pairs[1].x);
}


void CSVSpecFunction::CalcMinSpecDomain()
{
  minSpecDomain = pairs.begin()->x;
}

void CSVSpecFunction::CalcMaxSpecDomain()
{
  maxSpecDomain = (pairs.end() - 1)->x;
}

void CSVSpecFunction::CalcSpecRangeHead()
{
  specRangeHead = pairs.begin()->Fofx;
}

void CSVSpecFunction::CalcSpecRangeTail()
{
  specRangeTail = (pairs.end() - 1)->Fofx;
}

double CSVSpecFunction::GetMaxSpecX() const
{
  return maxSpecDomain;
}

double CSVSpecFunction::GetMinSpecX() const
{
  return minSpecDomain;
}

// TODO update this comment
// This is considered an elevator-style algorithm.
// We leverage the fact that the rotational velocity isn't going to change drastically
// from time-step to time-step. This means the current rotational velocity is close
// to the previous one; so we start our search from there. The upper bound
// for the complexity of this function (after the initial call) is therefore some constant, which depends on the timestep
// length and the type of inflows the turbine is experiencing. 
double CSVSpecFunction::F(double x) const
{
  // find entry below and above rotorSpeed, then interpolate

  if (x <= minSpecDomain) {
    return specRangeHead;
  }

  if (x >= maxSpecDomain) {
    return specRangeTail;
  }

  // not sure what to do in this case, but for now just return 0
  if (isnan(x)) {
    return 0;
  }

	
  unsigned int index = (unsigned int)floor((x - pairs[0].x) / deltaX);

  // going down
  if (x >= pairs[index].x) {
    for (unsigned int i = index; i < pairs.size(); ++i) {
      if (pairs[i].x > x) {
	// interpolate
	double result = Interpolate(pairs[index], pairs[i], x);

	// save the index so we can start from here for next time
	index = i;
	return result;
      }
      index = i;
    }
  }

  // going up
  else {
    for (unsigned int i = index; i >= 0; --i) {
      if (pairs[i].x < x) {
	// interpolate
	double result = Interpolate(pairs[index], pairs[i], x);

	return result;
      }
      index = i;
    }
  }

  // should never get here
  return 0.0;
}

double CSVSpecFunction::Interpolate(const Pair& a, const Pair& b, double x) const {
  const double epsilon = 1.0e-6;

  // if the rotorSpeed is close enough to the rotor speed of a,
  //    just return the generator torque corresponding to a
  if (fabs(x - a.x) < epsilon)
    return a.Fofx;
  // if the rotorSpeed is close enough to the rotor speed of b,
  //    just return the generator torque correspoinding to b
  if (fabs(x - b.x) < epsilon)
    return b.Fofx;

  // make sure we're not going to divide by something close to zero
  assert(fabs(b.x - a.x) > epsilon);

  // linearly interpolate
  return a.Fofx + (x - a.x) * (b.Fofx - a.Fofx) / (b.x - a.x);
}
