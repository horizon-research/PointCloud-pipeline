#include <string>  
#include <map>  
#include <iostream>  
#include <fstream>  
#include <sstream>  
#include <typeinfo>

// configuration file class
class Config
{

protected:
	// seperator between key and value
	std::string m_Comment;
	// seperator between value and comments
	std::string m_Delimiter;
	// extracted keys and values
	std::map<std::string, std::string> m_Contents;
	// map iterator
	typedef std::map<std::string, std::string>::const_iterator mapci;

public:

	Config(std::string filename, std::string delimiter = ":", std::string comment = "#");
	Config();

	template<class T>
	T Read(const std::string &in_key) const;

	template<class T>
	T Read(const std::string &in_key, const T &in_value) const;

private:

	friend std::istream &operator >> (std::istream &is, Config &cf);

	template<class T>
	static T string_as_T(const mapci &p);

	static void Trim(std::string &inout_s);

	struct File_Not_Found
	{
		std::string filename;
		File_Not_Found(const std::string &filename_ = std::string())
		:filename(filename_){}
	};

	struct Key_Not_Found
	{
		std::string key;
		Key_Not_Found(const std::string &key_ = std::string())
		:key(key_){}		
	};

};

template<class T>
T Config::string_as_T(const mapci &p)
{
	T t;
	std::istringstream ist(p->second);

	if (typeid(t) == typeid(bool)) // if key is bool type
	{
		ist >> std::boolalpha >> t;
		std::cout << p->first << " : " << t << std::endl;
		return t;
	}
	else if(ist >> t) // if key has a value
	{
		std::cout << p->first << " : " << t << std::endl;
		return t;
	}
	else
	{
		std::cout << "Value For Key " << p->first << " Is Invalid!" << std::endl;
		exit(-1);
	}
}

template<class T>
T Config::Read(const std::string &key, const T &value) const
{
	mapci p = m_Contents.find(key);
	if (p == m_Contents.end())
	{
		std::cout << "Key " << key << " Not Found!" << std::endl;
		throw Key_Not_Found(key);
	}
	else 
		return string_as_T<T>(p);
}