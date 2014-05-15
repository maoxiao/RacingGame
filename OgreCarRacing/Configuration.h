#ifndef __Configuration_h_
#define __Configuration_h_
//#define _USE_MATH_DEFINES

#include "iostream"
#include "rapidjson/document.h"		// rapidjson's DOM-style API
#include "rapidjson/prettywriter.h"	// for stringify JSON
#include "rapidjson/filestream.h"	// wrapper of C stream for prettywriter as output
#include <cstdio>
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

#include <math.h>



class Configuration
{
public:
	static Configuration * getInstance();
	~Configuration(void);
	static char* concatenate(const char *str1, const char *str2);
	Document mDocument;

private:

	
	Document::AllocatorType  mAllocator;

	
	char* readFile(const char* filename, size_t& length);
	void generate();
	void load(char *fileName);

	void logCarInfo();

	Configuration(void);
	static Configuration * m_pInstance;
	class Garbo//delete instance object
	{
	public:
		~Garbo()
		{
			if(Configuration::m_pInstance)
			{
				delete Configuration::m_pInstance;
			}
		}
	};
	static Garbo gb;
};

#endif
