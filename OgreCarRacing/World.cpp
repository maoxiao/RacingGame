// My header file.  This should go first!
#include "World.h"
#include "Ground.h"
#include "Configuration.h"
#include "Wall.h"

// Ogre header files
#include "Ogre.h"
#include "OgreMath.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "OgreOverlayManager.h"

#include "OgreFontManager.h"

// IOS (Input system) header files

#include <ois/ois.h>

// Other input files for my project
#include "Camera.h"
#include "InputHandler.h"
#include "PhysicsWorld.h"
#include "btBulletDynamicsCommon.h"
#include "StaticObject.h"

#define _USE_MATH_DEFINES
#include "math.h"


#include "./btogre/include/BtOgreExtras.h"
#include "./btogre/include/BtOgreGP.h"
#include "./btogre/include/BtOgreExtras.h"

float xMin, xMax, zMin, zMax;

#define Y_VALUE  5 



World::World(Ogre::SceneManager *sceneManager, InputHandler *input)   : mSceneManager(sceneManager), mInputHandler(input)
{

	level = -1;
	UICount = 0;
	// Global illumination for now.  Adding individual light sources will make you scene look more realistic
	mSceneManager->setAmbientLight(Ogre::ColourValue(1,1,1));	

	//create Player tank
	mPlayerCar = new Car(mSceneManager,"PlayerCar.mesh");
	
	carList.push_back(mPlayerCar);
	
	carList.push_back(new Car(mSceneManager,"AICube.mesh"));
	
	carList.push_back(new Car(mSceneManager,"AICube1.mesh"));
	

	//Create Environment
	new Ground(mSceneManager);
	soundManger = new Sound();

	//top wall
	new Wall(mSceneManager, 0, 0, -1000, 1000, 1000, 0, 1);

	//bottom wall
	new Wall(mSceneManager, 0, 0, 1000, 1000, 1000, 0, 2);

	//left wall
	new Wall(mSceneManager, 1000, 0, 0, 0, 1000, 1000, 3);

	//right wall
	new Wall(mSceneManager, -1000, 0, 0, 0, 1000, 1000, 4);

	createWorldProblems();
	//create startline

	Ogre::Entity *mStartline =SceneManager()->createEntity("Obstacle.mesh");
	startLineNode = SceneManager()->getRootSceneNode()->createChildSceneNode("startline", Ogre::Vector3(28*6,0,-32*6));
	startLineNode->attachObject(mStartline);
	startLineNode->scale(1*6, 1*2, 8*6);


	//create virltual startline
	Ogre::Entity *virtualStartLine =mSceneManager->createEntity("base.mesh");
	virtualStartLineNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	virtualStartLineNode->attachObject(virtualStartLine);

	virtualStartLineNode->setPosition(Ogre::Vector3(28*6,0,-75*6));
	virtualStartLineNode->scale(0.1, 10*2, 85*6);
	virtualStartLineNode->setVisible(false);


	//Create score board
	Ogre::OverlayManager& om = Ogre::OverlayManager::getSingleton();
	
	mInitialUIOverlay = om.getByName("InitialUI");
	mUIOverlay = om.getByName("PlayUI");

	mSpeedOverlay = om.getByName("Score");
	mFinalOverlay = om.getByName("Final");
	mShowUI = false;
	player1ScoreText = (Ogre::TextAreaOverlayElement *) om.getOverlayElement("Score/Panel1/Text1");
	mFinalOverlayText = (Ogre::TextAreaOverlayElement *) om.getOverlayElement("Final/Panel1/Text1");
	showInitUI();
	mGameOver = false;
	mShowFinalUI = false;
	//lapCollision = false;
}


//All player controls go into this method
void 
World::controlPlayer(float time)
{
	moveCar(time);
}



void 
World::moveCar(float time){

	
	if(mInputHandler->IsKeyDown(OIS::KC_UP) || mInputHandler->IsKeyDown(OIS::KC_DOWN) || 
	   mInputHandler->IsKeyDown(OIS::KC_RIGHT) || mInputHandler->IsKeyDown(OIS::KC_LEFT))
	{
		soundManger->playMoveSound();
	}

	bool linear = false;
	bool angular = false;

	btRigidBody *body = mPlayerCar->getRigidBody();

	if(mInputHandler->IsKeyDown(OIS::KC_UP))
	{
		mPlayerCar->move(true);
	}

		
	else if(mInputHandler->IsKeyDown(OIS::KC_DOWN))
	{
		mPlayerCar->move(false);
		
	} 

	//Turn the car
	if (mInputHandler->IsKeyDown(OIS::KC_LEFT))
	{
		angular = true;
		mPlayerCar->yaw(true);

	}
	else if (mInputHandler->IsKeyDown(OIS::KC_RIGHT))
	{
		angular = true;
		mPlayerCar->yaw(false);

	}
	mPlayerCar->applyBreak(0.07);
	

	
	if(!angular)
	{
		body->setAngularVelocity(btVector3(0,0,0));	
	}
	if (mPlayerCar->getTrackNum(level) < 0)
	{
		mPlayerCar -> slowDown();
	}

	mPlayerCar->updatePositionAndOrientation();

}

void 
createSpeedOverlay(){

}
void 
World::showSpeed(){
	Ogre::LogManager::getSingleton().logMessage("Show speed : " + Ogre::StringConverter::toString(mPlayerCar->getCarSpeed()) + " MPH");
	int lap = 0;
	if(mPlayerCar->mCurrentLap > 0){
		lap = mPlayerCar->mCurrentLap;
	}
	player1ScoreText->setCaption("Speed : " + Ogre::StringConverter::toString(int(mPlayerCar->getCarSpeed())) + " MPH" + 
		"\n Lap number: " + Ogre::StringConverter::toString(lap));
	mSpeedOverlay -> show();
}

void
World::hideSpeed(){
	mSpeedOverlay -> hide();
}
//Initial UI
void 
World::showInitUI(){
	Ogre::ResourceManager::ResourceMapIterator iter = Ogre::FontManager::getSingleton().getResourceIterator();
	while (iter.hasMoreElements()) 
	{ 
		iter.getNext()->load(); 
	}

	mShowInitialUI = true;
	
	mInitialUIOverlay->show();
	

}
//Handle UI commands

void 
World::handleUI(){

	if(mInputHandler->IsKeyDown(OIS::KC_1) && mShowUI){
		Car::level = level  = World::LEVE1;
		setupTracks(level);
		hideUI();
		mShowUI = false;
		mShowInitialUI = false;
	} else if(mInputHandler->IsKeyDown(OIS::KC_2) && mShowUI){
		Car::level = level  = World::LEVE2;
		setupTracks(level);
		hideUI();
		mShowUI = false;
		mShowInitialUI = false;
	} else if(mInputHandler->IsKeyDown(OIS::KC_1) && mShowInitialUI){
		Car::level = level  = World::LEVE1;
		setupTracks(level);
		hideInitialUI();
		mShowUI = false;
		mShowInitialUI = false;
	} else if(mInputHandler->IsKeyDown(OIS::KC_2) && mShowInitialUI){
		Car::level = level  = World::LEVE2;
		setupTracks(level);
		hideInitialUI();
		mShowUI = false;
		mShowInitialUI = false;
	}
}

void 
World::hideInitialUI(){
	mInitialUIOverlay->hide();
}
void 
World::hideUI(){
	mUIOverlay->hide();
}

void
World::showUI(){


	mShowUI = true;
	
	mUIOverlay->show();


}

void 
World::showFinalUI(){
	Ogre::LogManager::getSingleton().logMessage("Show final overlay" );
	mFinalOverlayText->setCaption(mPlayerCar->mWinner ? "Game over : \n The winner is You!! \n Congratulations!!! \n Press \"return\" key to coninue.." : "Game over : \n The winner is Computer \n Sorry!! \nBetter luck next time!!!  \n Press \"return\" key to coninue.." );
	mFinalOverlay -> show();
	mFinalOverlayText->show();
	
}

void 
World::hideFinalUI(){
	mFinalOverlay -> hide();
	mFinalOverlayText->hide();
	mShowFinalUI = false;
	mGameOver = false;
	showInitUI();
	
}

void
World::handleFinalUI(){
	if(mInputHandler->IsKeyDown(OIS::KC_RETURN)){
		hideFinalUI();

	}
}
//Think method
void 
World::Think(float time)
{
	if(mGameOver){
		if(!mShowFinalUI){
			mShowFinalUI = true;
			showFinalUI();
		} else{
			handleFinalUI();
		}
	} else {

		if(mInputHandler->IsKeyDown(OIS::KC_ESCAPE) && !mShowUI && !mShowInitialUI){
			UICount = UICount + 1;
			if(UICount > 10){
				showUI();
				mShowUI = true;
				UICount = 0;
			}
		
		} else if(mInputHandler->IsKeyDown(OIS::KC_ESCAPE) && mShowUI && !mShowInitialUI){
		
			UICount = UICount + 1;
			if(UICount > 10){
				hideUI();
				mShowUI = false;
				UICount = 0;
			}
		}
		if(mShowUI || mShowInitialUI){
			//Handle UI
			handleUI();
			hideSpeed();
		} else {
			PhysicsWorld::getInstance()->simulate(time);
			//Control Player
			controlPlayer(time);
			updateLaps();
			showSpeed();
		
		}
	}
	
	
	
}

void 
World::createWorldProblems(){

	//Create level1 tracks
	mObstacle = new StaticObject(mSceneManager, "Obstacle.mesh", Ogre::Vector3(56*6,2*6,1*6), BaseObject::WALL, "obstacle") ;
	mLevel1Track = new StaticObject(mSceneManager, "TrackBase0.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "semi_circle_track_1.0") ;
	mLevel1Track1 = new StaticObject(mSceneManager, "TrackBase1.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "straight_track_1.0") ;
	mLevel1Track2 = new StaticObject(mSceneManager, "TrackBase1.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "straight_track_1.1") ;
	mLevel1Track3 = new StaticObject(mSceneManager, "TrackBase0.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "semi_circle_track_1.1") ;
	
	mObstacle2 = new StaticObject(mSceneManager, "Obstacle.mesh", Ogre::Vector3(56*6,2*6,1*6), BaseObject::WALL, "obstacle2") ;

	//Create level2 tracks
	mLevel2Track = new StaticObject(mSceneManager, "TrackBase0.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "semi_circle_track_2.0") ;
	mLevel2Track1 = new StaticObject(mSceneManager, "TrackBase2.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "quarter_circle_track_2.0") ;
	mLevel2Track2 = new StaticObject(mSceneManager, "TrackBase0.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "semi_circle_track_2.1") ;
	mLevel2Track3 = new StaticObject(mSceneManager, "TrackBase1.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "straight_track_2.0") ;
	mLevel2Track4 = new StaticObject(mSceneManager, "TrackBase2.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "quarter_circle_track_2.1") ;
	mLevel2Track5 = new StaticObject(mSceneManager, "TrackBase1.mesh", Ogre::Vector3(6.3,6,6), BaseObject::TRACK, "straight_track_2.1") ;
}


void
World::setupTracks(int iLevel)
{

	level = iLevel;

	//level1
	if(level == World::LEVE1)
	{
		
	
		//Set position for level1 tracks
		mObstacle->setPosition(btVector3(0, 0, 0));

		mLevel1Track->setPosition(btVector3(-32 * 6, 0, 0));
		trackList.push_back(mLevel1Track);

		
		mLevel1Track1->setPosition(btVector3(0, 0, 32*6));
		trackList.push_back(mLevel1Track1);
		
		mLevel1Track2->setPosition(btVector3(0, 0, -32*6));
		trackList.push_back(mLevel1Track2);

		mLevel1Track3->setPosition(btVector3(32*6, 0, 0));
		mLevel1Track3->setRotation(btScalar (M_PI));
		trackList.push_back(mLevel1Track3);
		
		setUpCars();

		for (int i = 0; i < carList.size(); i++)
		{
			carList[i]->reset();
		}
		//Hide level2 tracks
		mObstacle2->setPosition(btVector3(32*6, -20, 32*6));
		mLevel2Track->setPosition(btVector3(-32 * 6, -20, 0));
		mLevel2Track1->setPosition(btVector3(-32 * 6, -20, 64  *6));
		mLevel2Track2->setPosition(btVector3(32 * 6, -20, 64 * 6));
		mLevel2Track3->setPosition(btVector3(64 * 6, -20, 32 * 6));
		mLevel2Track4->setPosition(btVector3(32 * 6, -20, 0));
		mLevel2Track5->setPosition(btVector3(1 *6 , -20, -32 * 6));
	}

	else
	{

		//Set position for track2
		
		mObstacle->setPosition(btVector3(0, 0, 0));

		mObstacle2->setPosition(btVector3(32*6, 0, 32*6));
		mObstacle2->setRotation(M_PI/2);


		//mTrack = new StaticObject(mSceneManager, "TrackBase0.mesh", Ogre::Vector3(6,6,6), BaseObject::TRACK, "semi_circle_track_0") ;
		mLevel2Track->setPosition(btVector3(-32 * 6, 0, 0));
		trackList.push_back(mLevel2Track);

		mLevel2Track1->setPosition(btVector3(-32 * 6, 0, 64  *6));
		mLevel2Track1->setRotation(btScalar (M_PI));
		trackList.push_back(mLevel2Track1);
	
		mLevel2Track2->setPosition(btVector3(32 * 6, 0, 64 * 6));
		mLevel2Track2->setRotation(btScalar (M_PI/2));
		trackList.push_back(mLevel2Track2);
	
		mLevel2Track3->setPosition(btVector3(64 * 6, 0, 32 * 6));
		mLevel2Track3->setRotation(btScalar (M_PI/2));
		trackList.push_back(mLevel2Track3);

		mLevel2Track4->setPosition(btVector3(32 * 6, 0, 0));
		mLevel2Track4->setRotation(btScalar (M_PI));
		trackList.push_back(mLevel2Track4);

		mLevel2Track5->setPosition(btVector3(1 *6 , 0, -32 * 6));
		trackList.push_back(mLevel2Track5);

		setUpCars();
		

		for (int i = 0; i < carList.size(); i++)
		{
			carList[i]->reset();
		}
		//Hide track1
		mLevel1Track->setPosition(btVector3(-32 * 6, -20, 0));
		mLevel1Track1->setPosition(btVector3(0, -20, 32*6));
		mLevel1Track2->setPosition(btVector3(0, -20, -32*6));
		mLevel1Track3->setPosition(btVector3(0, -20, -32*6));
	}
}

void
World::setUpCars()
{
	mPlayerCar->setPosition(btVector3(-32 * 6,Y_VALUE,-32*6));
	mPlayerCar->setRotation(M_PI/2);

	carList[1] -> setPosition(btVector3(-12 * 6, Y_VALUE, -32*6));
	carList[1] -> setRotation(M_PI/2);

	carList[2] -> setPosition(btVector3(8 * 6, Y_VALUE, -32*6));
	carList[2] -> setRotation(M_PI/2);
}
//check startline collision
void
World::updateLaps(){

	bool collision;
	for(int i =0; i<carList.size(); i++){
		Ogre::AxisAlignedBox aab = virtualStartLineNode->_getWorldAABB().intersection((carList[i]->getSceneNode())->_getWorldAABB());
		collision = (!aab.isNull());
		
		Ogre::LogManager::getSingleton().logMessage("lapCollision: " + Ogre::StringConverter::toString(carList[i]->lapCollision) );
		Ogre::LogManager::getSingleton().logMessage("collision boy: " + Ogre::StringConverter::toString(collision) );
		Ogre::LogManager::getSingleton().logMessage("Track number: " + Ogre::StringConverter::toString(carList[i]->getTrackNum(level)) );
		Ogre::LogManager::getSingleton().logMessage("Car id: " + Ogre::StringConverter::toString(carList[i]->carId) );
		Ogre::LogManager::getSingleton().logMessage("Level : " + Ogre::StringConverter::toString(level) );
		if(level == 0){
			if(collision && carList[i]-> mForward && !carList[i]->lapCollision && carList[i]->getTrackNum(level) == 2){
			carList[i]->lapCollision = true;
			
			carList[i]->mCurrentLap = carList[i]->mCurrentLap + 1;
			if(carList[i]->mCurrentLap == 3){
				mGameOver = true;
				carList[i]->mWinner = true;
			}
			
			}
			if(carList[i]->getTrackNum(level) != 2 && carList[i]->lapCollision == true){
				carList[i]->lapCollision = false;
			}
		} else {
			if(collision && carList[i]-> mForward && !carList[i]->lapCollision && carList[i]->getTrackNum(level) == 5){
			carList[i]->lapCollision = true;
			
			carList[i]->mCurrentLap = carList[i]->mCurrentLap + 1;
			if(carList[i]->mCurrentLap == 3){
				mGameOver = true;
				carList[i]->mWinner = true;
			}
			
			}
			if(carList[i]->getTrackNum(level) != 5 && carList[i]->lapCollision == true){
				carList[i]->lapCollision = false;
			}
		}
		
	}

	
}