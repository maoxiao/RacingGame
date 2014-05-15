#ifndef __StaticObject_h_
#define __Ground_h_

#include "baseobject.h"


class StaticObject :
	public BaseObject
{
public:
	StaticObject(Ogre::SceneManager *sceneManager, std::string meshName, Ogre::Vector3 scale, int type, std::string objectName);
	~StaticObject(void);

private:
	static int count;
};

#endif
