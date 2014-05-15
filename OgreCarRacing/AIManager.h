#ifndef __AIManager_h_
#define __AIManager_h_

#include "World.h"

class World;
class Car;
class ProjectileManger;

namespace Ogre {
    class SceneNode;
    class SceneManager;

}
class AIManager 
{

public:
    // You may want to add parameters to the AI Manager constructor
	AIManager(World *world, Ogre::SceneManager *sceneManager);
	~AIManager();

    // Go through all of the AIs, and call think on each one
    void Think(float time);

protected:
	World *mWorld;
	//ProjectileManger *mProjectileManger;

	Ogre::SceneManager *mSceneManager;

    // Probablay want a list of individual AI classes here ...
};

#endif