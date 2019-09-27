#include "InputFile.h"
#include <sstream>
#include <fstream>
#include <string>

class FileContentsException : public std::runtime_error {
public:
	FileContentsException(const std::string& errMsg) : std::runtime_error(errMsg) {}
};

class FileNotFoundException : public std::runtime_error {
public:
	FileNotFoundException(const std::string& errMsg) : std::runtime_error(errMsg) {}
};

InputFile::InputFile()
{

}

void InputFile::LoadFile(const char* fname)
{
	fin.open(fname, std::fstream::in);
	if (!fin.is_open()) {
		std::string errMsg = std::string("Couldn't open input file: ") + std::string(fname);
		throw FileNotFoundException(errMsg);
	}
}

void InputFile::CloseFile()
{
	fin.close();
}

// Reads the next non-comment line, converts it to a number, 
// and assigns it to the passed-by-reference parameter, value. If the label doesn't match
// what in the input file at this point, or the read-in value is not a valid 
// number, then it returns false; but otherwise returns true.
double InputFile::ReadDouble(const char* label)
{
	std::string line = GetNextNonCommentLine();

	std::stringstream linestream(line);
	std::string token;

	// get LHS of assignment
	std::getline(linestream, token, '=');
	if (token != label) {
		std::string errMsg = std::string("Expected ") + std::string(label) +
			std::string(" in input file but got ") + std::string(token);
		throw FileContentsException(errMsg);
	}

	// get RHS of assignment
	std::getline(linestream, token);
	if (!IsNumber(token)) {
		std::string errMsg = ("Expected a decimal number but got ") + std::string(token);
		throw FileContentsException(errMsg);
	}

	return std::stod(token.c_str());
}

std::string InputFile::GetNextNonCommentLine()
{
	// throw away every comment line until we reach a non-comment line, and then return it
	std::string line;
	while (getline(fin , line, '\n')) {
		if (line.front() != '!')
			break;
	}

	return line;
}

// Note, will return false for numbers written in scientific notation, e.g.  "1.0e-06"
bool InputFile::IsNumber(const std::string& s) const
{
	bool decimalFound = false;
	for (std::string::const_iterator it = s.begin(); it != s.end(); it++) {
		// if this character is a decimal
		if ((*it) == '.') {
			// if a decimal has already been found, then there are two decimals
			// so this is not a valid number
			if (decimalFound) {
				return false;
			}
			decimalFound = true;
		}

		// otherwise, if this character isn't a valid digit
		else if (!std::isdigit(*it)) {
			return false;
		}
	}
	return true;
}
