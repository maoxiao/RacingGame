
#include <stddef.h>
#include <string>
#include "Configuration.h"

#include "Ogre.h"

//#define PI 3.1415926

Configuration *Configuration::m_pInstance = NULL;

Configuration::Configuration(void)
{  
	

	mAllocator = mDocument.GetAllocator();
	const char json[] = "{}";
	

	if (mDocument.Parse<0>(json).HasParseError())
	{
		return;
	}

	//generate();
	load("configuration.json");
	logCarInfo();
}

Configuration*
Configuration::getInstance()
{
	if(m_pInstance==NULL)
		m_pInstance=new Configuration();
	return m_pInstance;
}


char*
Configuration::concatenate(const char *str1, const char *str2)
{
	int length = sizeof(str1) + sizeof(str2);
	char *result = new char [length];
	std::string s = str1;
	s.append(str2);
			
	memcpy(result, s.c_str(), length);
	
	return result;
}

char* 
Configuration::readFile(const char* filename, size_t& length) {
	FILE *fp = fopen(filename, "rb");

	if (!fp)
		fp = fopen(filename, "rb");
	if (!fp)
		return 0;

	fseek(fp, 0, SEEK_END);
	length = (size_t)ftell(fp);
	fseek(fp, 0, SEEK_SET);
	char* json = (char*)malloc(length + 1);
	fread(json, 1, length, fp);
	json[length] = '\0';
	fclose(fp);
	return json;
}

void
Configuration::generate()
{
	Value basicCarData;
	{
		basicCarData.SetObject();
		basicCarData.AddMember("carNum", 4, mAllocator);
		basicCarData.AddMember("initialSpeed", 30.0, mAllocator);
		basicCarData.AddMember("speedLimit", 50.0, mAllocator);
	}
	mDocument.AddMember("basicCarData", basicCarData, mAllocator);

	Value carList;
	{
		carList.SetObject();
		int num = mDocument["basicCarData"]["carNum"].GetInt();

		for(int i = 0; i < num; i++) 
		{
			Value v;
			v.SetObject();
			Value position;
			position.SetArray();
			position.PushBack(0, mAllocator);
			position.PushBack(0, mAllocator);
			position.PushBack(0, mAllocator);

			v.AddMember("position", position, mAllocator);
			
			Value meshName;
			v.AddMember("meshName", "name", mAllocator);

			Value rotation;
			v.AddMember("rotation", 0.0, mAllocator);
			
			char *key = concatenate("car", std::to_string(i).c_str());
			carList.AddMember(key, v, mAllocator);
		}

	}

	mDocument.AddMember("car_list", carList, mAllocator);

	
	GenericStringBuffer< UTF8<> >  outBuffer;
	//Writer< GenericStringBuffer< UTF8<> > > writer(outBuffer);
	PrettyWriter< GenericStringBuffer< UTF8<> > > writer(outBuffer);
	mDocument.Accept(writer);	// Accept() traverses the DOM and generates Handler events.
	
	FILE * wFile = fopen ("configuration.json" , "w");
	fwrite (outBuffer.GetString(), outBuffer.Size(), 1, wFile);
	fclose(wFile);
}



void
Configuration::load(char *fileName)
{
	size_t length;
	char* json = readFile(fileName, length);
	if (mDocument.Parse<0>(json).HasParseError())
	{
		return;
	}
}

//an example for get json value
void 
Configuration::logCarInfo()
{
	int num = 0;
	
	if(mDocument["basicCarData"]["carNum"].IsInt() )
	{	
		//get int value
		num = mDocument["basicCarData"]["carNum"].GetInt();
	}
	Ogre::LogManager::getSingleton().logMessage("carNum: " + Ogre::StringConverter::toString(num));
	
	
	if(mDocument["car_list"]["car0"]["position"].IsArray())
	{
		//get array values
		Value &array = mDocument["car_list"]["car0"]["position"];

		//use SizeType instead of int for array index
		double x = array[SizeType(0)].GetDouble();
		double y = array[SizeType(1)].GetDouble();
		double z = array[SizeType(2)].GetDouble();

		//get string value using char*
		const char * name = mDocument["car_list"]["car0"]["meshName"].GetString();
		Ogre::LogManager::getSingleton().logMessage("car0 meshName: " + std::string(name));
		//get string value using std::string
		//std::string name = mDocument["car_list"]["car0"]["meshName"].GetString();
		//Ogre::LogManager::getSingleton().logMessage("car0 meshName: " + Ogre::StringConverter::toString(name));
		
		
		Ogre::LogManager::getSingleton().logMessage("car0 position: " + Ogre::StringConverter::toString(Ogre::Vector3(x,y,z)));
	}
	
}

Configuration::~Configuration(void)
{
}
