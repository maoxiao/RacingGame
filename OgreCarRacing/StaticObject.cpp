#include "StaticObject.h"

int StaticObject::count = -1;

StaticObject::StaticObject(Ogre::SceneManager *sceneManager, std::string meshName, Ogre::Vector3 scale, int type, std::string objectName):
							BaseObject(
							sceneManager, 
							meshName,				//mesh file name
							scale,					//scale
							type,					//type of object
							0.0f,					//mass
							objectName,				//object name
							count					//object id
							)
{
	count++;

}


StaticObject::~StaticObject(void)
{
}
