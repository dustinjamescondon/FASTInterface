/*
File: InputFile.h
=================

Author: Dustin Condon
=====================

Date: Nov 2019
==============





Description
===========

  This class provides a method a reading values from a plain-text
  file. The Read functions take a label string. The given file is read
  from the top to bottom, and each value read in must be read in order.

*/

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