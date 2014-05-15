#include "AIManager.h"
#include <cstdlib>
#include "OgreSceneNode.h"
//#include "Car.h"
#include "Ogre.h"


#define DIRECTION_TIMER_MAX_VALUE 1.0f
#define ROTATE_RATE 0.5f
#define ROTATE_TIME 1.0f

AIManager::AIManager(World *world, Ogre::SceneManager *sceneManager) : mWorld(world), mSceneManager(sceneManager)
{
    // Any initialization that needs to be done ...
	
}

AIManager::~AIManager()
{
    // Clean up after yourself ...
}

void 
AIManager::Think(float time)
{
	for (int i = 1; i < MAX_ENEMY_CAR + 1; i++)
	{
	
		Car *car = mWorld -> carList[i];
		car->autoDrive();
		car->updatePositionAndOrientation();
	}
}

