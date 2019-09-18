#pragma once
#include <stdexcept>

class FileContentsException : public std::runtime_error {
public:
	FileContentsException(const std::string& errMsg) : std::runtime_error(errMsg) {}
};

class FileNotFoundException : public std::runtime_error {
public:
	FileNotFoundException(const std::string& errMsg) : std::runtime_error(errMsg) {}
};
