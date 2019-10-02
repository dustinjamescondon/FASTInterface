#pragma once

#include <string>
#include <fstream>

class InputFile {
public:
	InputFile();

	void LoadFile(const char*);
	void CloseFile();

	double ReadDouble(const char* label);
	double ReadDouble(std::ifstream& fin, const char* label);

private:
	std::string GetNextNonCommentLine();
	bool IsNumber(const std::string&) const;

	std::ifstream fin;
};