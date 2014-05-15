#ifndef __World_h_
 
#define __World_h_
#include "OgreVector3.h"
#include "Car.h"
#include <string>
#include "Sound.h"
#include "OgreOggSound.h"
#include "OgreOverlay.h"
#include "OgreTextAreaOverlayElement.h"


#define MAX_ENEMY_CAR 2


// Forward delcarations of Ogre classes.  Note the Ogre namespace!
namespace Ogre {
    class SceneNode;
    class SceneManager;

}

// Forward delcarations of our own classes
class PongCamera;
class InputHandler;
class vector;
class StaticObject;



class World
{
public:
	enum Type
	{
		LEVE1,
		LEVE2
	};
	
	Car *mPlayerCar;

    World(Ogre::SceneManager *sceneManager, InputHandler *input);
    void Think(float time);
	Ogre::SceneManager *SceneManager() { return mSceneManager; }
	void World::attachCameraToTank(Ogre::Camera *mPlayerCam);
	void World::attachCameraToTank(std::string input);
	void World::detachCameraFromTank();
	void createGround();
	void controlPlayer(float time);
	void World::moveCar(float time);
	std::vector<Car*> carList;
	std::vector<StaticObject*> trackList;
	Sound *soundManger;
	void createWorldProblems();
	void showInitUI();
	void showUI();
	void hideUI();
	void handleUI();
	void setupTracks(int level);
	Ogre::SceneNode *startLineNode;
	void showSpeed();
	void hideSpeed();
	void createSpeedOverlay();
	
	void hideInitialUI();
	bool mShowUI, mShowInitialUI;

	void World::updateLaps();
	Ogre::SceneNode *virtualStartLineNode;
	void setUpCars();
	void showFinalUI();
	void hideFinalUI();
	void handleFinalUI();
protected:
	bool mGameOver, mShowFinalUI;
	StaticObject *mLevel1Track;

	StaticObject *mLevel1Track1;

	StaticObject *mLevel1Track2;

	StaticObject *mLevel1Track3;

	StaticObject *mLevel2Track;

	StaticObject *mLevel2Track1;

	StaticObject *mLevel2Track2;

	StaticObject *mLevel2Track3;

	StaticObject *mLevel2Track4;

	StaticObject *mLevel2Track5;

	//bool lapCollision;
	//Obstacle
	StaticObject *mObstacle;
	StaticObject *mObstacle1;
	StaticObject *mObstacle2;

	Ogre::Overlay *mSpeedOverlay;
	Ogre::TextAreaOverlayElement *pCarSpeed;
	Ogre::Overlay *mUIOverlay,*mInitialUIOverlay, *mFinalOverlay;
	Ogre::TextAreaOverlayElement *player1ScoreText, *mFinalOverlayText;
	int UICount;
	Ogre::SceneManager *mSceneManager;
	InputHandler *mInputHandler;
	int level;
	Ogre::Camera *mPlayerCam;
};

#endif
