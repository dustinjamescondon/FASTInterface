#pragma once

#include <stdexcept>

// exception class for any other AeroDyn error
class ADErrorException : public std::runtime_error {
public:
	inline ADErrorException(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the AeroDyn input file cannot be found
class ADInputFileNotFoundException : public std::runtime_error {
public:
	inline ADInputFileNotFoundException(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the AeroDyn input file has invalid contents
class ADInputFileContentsException : public std::runtime_error {
public:
	inline ADInputFileContentsException(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the Bladed DLL could not be loaded at the specified path
class BladedDLLNotFoundException : public std::runtime_error {
public:
	inline BladedDLLNotFoundException(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the main DISCON procedure in the bladed-style DLL couldn't be loaded
class BladedDLLProcedureNotFoundException : public std::runtime_error {
public:
	inline BladedDLLProcedureNotFoundException(const char* errMsg) : std::runtime_error(errMsg) {}
};
