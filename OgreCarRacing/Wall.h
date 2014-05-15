#ifndef __Wall_h_
#define __Wall_h_

#include "baseobject.h"

class Wall : public BaseObject
{
public:
	Wall(Ogre::SceneManager *sceneManager, float originX, float originY, float orininZ, float scaleX, float scaleY, float scaleZ, int type);
	~Wall(void);
};


#endif
