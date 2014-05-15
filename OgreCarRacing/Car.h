#ifndef __CAR_h_
#define __CAR_h_

#include "OgreMatrix3.h"
#include "OgreVector3.h"
#include "btBulletDynamicsCommon.h"

#include <string>

#include "BaseObject.h"
#include "Configuration.h"

class Ogre::SceneManager;
class Ogre::SceneNode;



class Car: public BaseObject
{
public:
	Car(Ogre::SceneManager *sceneManager);
	Car(Ogre::SceneManager *sceneManager, std::string meshName);
	void move(bool foward);
	void yaw(bool left);
	void updateCarStats(bool Forward);
	void reset();
	bool applyBreak(float value);
	void positionBeforeCollistion();
	void autoDrive();
	float getDistance(btVector3 &carPos, btVector3 &point);
	int getTrackNum(int level);
	bool checkStraight(Value &track, float scale, float width, int angle);
	bool checkCurve(Value &track, float scale, float width, int angle);
	float getLargerValue(float v1, float v2) { return v1 > v2 ? v1 : v2; };
	float getSmallerValue(float v1, float v2) { return v1 < v2 ? v1 : v2; };
	static int level;
	
	bool isOnTheGround();
	void slowDown();

	float getCarSpeed();
	bool mForward;
	int mCurrentLap;
	int carId;
	bool lapCollision;
	bool mWinner;
	void moveOnStraightTrack(Value &track);
	void moveOnCurveTrack(Value &track);
	float calculateAngularVeloctiyOnCurve(Value &track, float radius);
	float getDistanceFromCurveCenter(Value &track);

protected:
	bool mLeft;

private:
	static int count;
	float mMaxSpeed;
	float mCurrentSpeed;
	float mSpeedLimit;
	
	btTransform mTransformBeforeCollision;
	float mAngularVelocity;

	int oldTrackNum;
	float initialRadiusForEntryPoint;
};



#endif
