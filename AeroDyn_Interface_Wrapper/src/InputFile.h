#pragma once

#include <string>
#include <fstream>

class InputFile {
public:
	InputFile();

	void Load(const char*);
	void Close();

	double ReadDouble(const char* label);
	double ReadDouble(std::ifstream& fin, const char* label);
	std::string ReadString(const char* label);
	std::string ReadString(std::ifstream& fin, const char* label);

private:
	std::string GetNextNonCommentLine();
	bool IsNumber(const std::string&) const;

	std::ifstream fin;
	std::string filename;
};