#include "read_config.h"

Config::Config(std::string filename, std::string delimiter, std::string comment) \
	:m_Delimiter(delimiter), m_Comment(comment)
{
	std::ifstream in (filename.c_str());

	if (!in)
		throw File_Not_Found(filename);

	in >> (*this);
}

// remove leading and trailing white space
void Config::Trim(std::string &inout_s)
{
	const char white_space[] = "\n\t\v\r\f";
	inout_s.erase(0, inout_s.find_first_not_of(white_space));
	inout_s.erase(inout_s.find_last_not_of(white_space) + 1U);
}

std::istream &operator>>(std::istream &is, Config &cf)
{
	typedef std::string::size_type pos;
	const std::string &delim = cf.m_Delimiter;
	const std::string &comm = cf.m_Comment;
	const pos skip = delim.length();

	std::string line;

	while (std::getline(is, line))
	{
		// ignore comments
		line = line.substr(0, line.find(comm));

		pos delimPos = line.find(delim);
		// find a delimiter before termination of string
		if (delimPos < std::string::npos)
		{
			// extract key
			std::string key = line.substr(0, delimPos);
			line.replace(0, delimPos + skip, "");

			Config::Trim(key);
			Config::Trim(line);

			cf.m_Contents[key] = line;
		}
	}
	return is;
}