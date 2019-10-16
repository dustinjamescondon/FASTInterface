#pragma once

#include <stdexcept>

// exception class for any other AeroDyn error
class ADErrorException : public std::runtime_error {
public:
	inline ADErrorException(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the AeroDyn input file cannot be found
class FileNotFoundException : public std::runtime_error {
public:
	inline FileNotFoundException(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the AeroDyn input file has invalid contents
class FileContentsException : public std::runtime_error {
public:
	inline FileContentsException(const char* errMsg) : std::runtime_error(errMsg) {}
};