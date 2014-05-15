
#include "Car.h"
#include "OgreSceneManager.h"
#include "OgreStringConverter.h"
#include "OgreSceneNode.h"
#include "OgreEntity.h"
#include "OgreQuaternion.h"
#include "OgreMath.h"



#include "./btogre/include/BtOgreExtras.h"
#include "./btogre/include/BtOgreGP.h"
#include "./btogre/include/BtOgreExtras.h"
#include "PhysicsWorld.h"

#include <limits>


#define _USE_MATH_DEFINES
#include "math.h"

int Car::count = 0;
int Car::level = -1;

//Constructor
Car::Car(Ogre::SceneManager *sceneManager):
		BaseObject(sceneManager, 
				   std::string("Body.mesh"),					//mesh file name
				   Ogre::Vector3(3,3,3),					    //scale
				   BaseObject::CAR,								//type of object
				   20.0f,										//mass
				   std::string("car" + std::to_string(count)),	//object name
				   count										//object id
				   )										
{
	Ogre::LogManager::getSingleton().logMessage("carID: " + Ogre::StringConverter::toString(mID));	
	mTransformBeforeCollision = mRigidBody->getWorldTransform();
	mRigidBody->setAngularFactor(btVector3(0,1,0));
	count++;
	reset();
	
}


Car::Car(Ogre::SceneManager *sceneManager, std::string meshName):
		BaseObject(sceneManager, 
				   meshName,				//mesh file name
				   Ogre::Vector3(3,3,3), //scale
				   BaseObject::CAR,			//type of object
				   100.0f,					//mass
				   std::string("car" + std::to_string(count)),				//object name
				   count					//object id
				  )
{
	Ogre::LogManager::getSingleton().logMessage("carID: " + Ogre::StringConverter::toString(mID));
	mTransformBeforeCollision = mRigidBody->getWorldTransform();
	mRigidBody->setAngularFactor(btVector3(0,1,0));
	count++;
	reset();
}


void
Car::positionBeforeCollistion(){
	if(collideWithType == BaseObject::CAR)
	{
		mTransformBeforeCollision = mRigidBody->getWorldTransform();
	}
}

void
Car::updateCarStats(bool forward)
{
	btVector3 carVelocity = mRigidBody->getLinearVelocity();
	mCurrentSpeed = carVelocity.length();
	mForward = forward;
	
}

float 
Car::getCarSpeed()
{
	return mCurrentSpeed;
}
bool
Car::applyBreak(float value)
{
	if(mCurrentSpeed > 0.5){
		Ogre::Vector3 direction = mSceneNode->getOrientation() * Ogre::Vector3::UNIT_Z;
		mCurrentSpeed = mCurrentSpeed - value;
		btVector3 velocity = btVector3(0,0,0);
		if(mForward){
			
			velocity = btVector3(direction.x, 0, direction.z) * (mCurrentSpeed);
		} else {
			velocity = btVector3(direction.x, 0, direction.z) * (-mCurrentSpeed);
		}
		
		mRigidBody->activate();
		mRigidBody->setLinearVelocity(velocity);
		return true;
	} else {
		return false;
	}
	
}

void
Car::move(bool foward)
{
	
	if(getTrackNum(level) < 0)
	{
		mMaxSpeed = 20;
	} 
	if(getTrackNum(level) > -1){
		Document &document = Configuration::getInstance()->mDocument;
		Value &car = document["car_list"][Configuration::concatenate("car", std::to_string(mID).c_str())];
		mMaxSpeed = car["maxSpeed"].GetInt();
	}
	if(foward != mForward && mCurrentSpeed > 0.0f){
		//Stop the car first before taking any action
		bool carStillMoving = applyBreak(0.8);
		Ogre::LogManager::getSingleton().logMessage("Applying break : speed is :  " + Ogre::StringConverter::toString(mCurrentSpeed));
		if(!carStillMoving){
			Ogre::LogManager::getSingleton().logMessage("Car is not moving " );
			updateCarStats(foward);
		}
	} 
	

	if(mCurrentSpeed < mMaxSpeed){ 
		updateCarStats(foward);
		Ogre::Vector3 direction = mSceneNode->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
		direction.normalise();
		Ogre::LogManager::getSingleton().logMessage("mCarSpeed  " + Ogre::StringConverter::toString(mCurrentSpeed));
		mCurrentSpeed = mCurrentSpeed + .5;
		btVector3 velocity = btVector3(direction.x, 0, direction.z) * (foward ? mCurrentSpeed: -mCurrentSpeed);
		mRigidBody->activate();
		mRigidBody->setLinearVelocity(velocity);
	}
	else if (mCurrentSpeed > mMaxSpeed) {
		updateCarStats(foward);
		Ogre::Vector3 direction = mSceneNode->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
		direction.normalise();
		Ogre::LogManager::getSingleton().logMessage("mCarSpeed  " + Ogre::StringConverter::toString(mCurrentSpeed));
		mCurrentSpeed = mCurrentSpeed - .5;
		btVector3 velocity = btVector3(direction.x, 0, direction.z) * (foward ? mCurrentSpeed: -mCurrentSpeed);
		mRigidBody->activate();
		mRigidBody->setLinearVelocity(velocity);
	}
}

void 
Car::slowDown()
{
	mMaxSpeed =  20.f;
}

bool 
Car::isOnTheGround() 
{ 
	return Car::collideWithType == BaseObject::GROUND; 
}


void
Car::yaw(bool left)
{
	updateCarStats(mForward);
	
	if(mCurrentSpeed > 3){
		mRigidBody->activate();
		float rotate_rate = 0.2f;

		if (mLeft != left)
		{
			mRigidBody ->setAngularVelocity(btVector3(0, 0, 0));
			mLeft = !mLeft;
		}
		
		float maxAngularVelocity = 1.0f;
		if(mCurrentSpeed < 18){
			maxAngularVelocity = 0.1f;
		}
		if(mCurrentSpeed < 10){
			maxAngularVelocity = 0.07f;
		}
		if(mForward && left){
			mAngularVelocity = mAngularVelocity + 0.1f;
		} else if(mForward && !left){
			mAngularVelocity = mAngularVelocity - 0.1f;
		}else if(!mForward && left){
			//Turn right
			mAngularVelocity = mAngularVelocity - 0.1f;
		} else if(!mForward && !left){
			//Turn Left
			mAngularVelocity = mAngularVelocity + 0.1f;
		}
		if (mAngularVelocity > maxAngularVelocity)
		{
			mAngularVelocity = maxAngularVelocity;
		}
		if (mAngularVelocity < -maxAngularVelocity)
		{
			mAngularVelocity = -maxAngularVelocity;
		}
	
		mRigidBody->setAngularVelocity(btVector3(0, mAngularVelocity, 0));
	}
}


float
Car::getDistance(btVector3 &carPos, btVector3 &point)
{
	
	return sqrt(pow(carPos.getX() - point.getX(), 2) 
			  + pow(carPos.getY() - point.getY(), 2) 
			  + pow(carPos.getZ() - point.getZ(), 2));
}

float 
Car::getDistanceFromCurveCenter(Value &track)
{
	btVector3 carPos = mRigidBody->getWorldTransform().getOrigin();

	float scale = track["scale"].GetDouble();
	
	Value &trackPosArray = track["postion"];
	float x = scale * trackPosArray[SizeType(0)].GetDouble();
	float y = scale * trackPosArray[SizeType(1)].GetDouble();
	float z = scale * trackPosArray[SizeType(2)].GetDouble();
	return getDistance(btVector3(carPos.getX(), y, carPos.getZ()), btVector3(x,y,z));;
}

float 
Car::calculateAngularVeloctiyOnCurve(Value &track, float radius)
{
	int turning = track["turning"].GetInt();
	return 1 * turning * mCurrentSpeed / radius;	
}

void 
Car::moveOnStraightTrack(Value &track)
{
	int axis = track["parallel_to"].GetInt();
	btVector3 carPos = mRigidBody->getWorldTransform().getOrigin();
	float distance;

	//1 == axis parallel to x;  3 == axis parallel to z
	distance = (1 == axis) ? abs(carPos.getZ()) : abs(carPos.getX());
	Ogre::LogManager::getSingleton().logMessage("check carx: " + Ogre::StringConverter::toString(carPos.getX()) );

	Ogre::Vector3 direction = mSceneNode->getOrientation() * Ogre::Vector3::UNIT_Z;
	btVector3 velocity = btVector3(direction.x, 0, direction.z).normalize() * mCurrentSpeed;
	mRigidBody->activate();
	mRigidBody->setLinearVelocity(velocity);

	float scale = track["scale"].GetDouble();
	Value &trackPosArray = track["postion"];
	float x = scale * trackPosArray[SizeType(0)].GetDouble();
	float y = scale * trackPosArray[SizeType(1)].GetDouble();
	float z = scale * trackPosArray[SizeType(2)].GetDouble();
	float controlDistance = (1 == axis) ? abs(z) : abs(x) ;


	float d = abs(distance - controlDistance);
	if (d > 0.1)
	{
		if (distance > controlDistance)
		{
			mAngularVelocity =  abs(mAngularVelocity)- 0.00001 * d;
		}
		else
		{
			mAngularVelocity =  abs(mAngularVelocity) + 0.00001 * d;
		}

	}
	else
	{
		mAngularVelocity = 0.f;
	}
	mRigidBody->setAngularVelocity(btVector3(0, mAngularVelocity, 0));

}

void 
Car::moveOnCurveTrack(Value &track)
{

	Ogre::Vector3 direction = mSceneNode->getOrientation() * Ogre::Vector3::UNIT_Z;
	btVector3 velocity = btVector3(direction.x, 0, direction.z).normalize() * mCurrentSpeed;
	mRigidBody->activate();
	mRigidBody->setLinearVelocity(velocity);

	float currentRadius = getDistanceFromCurveCenter(track);
	float d = abs(currentRadius - initialRadiusForEntryPoint);
	float initialAngularVelocity = calculateAngularVeloctiyOnCurve(track, initialRadiusForEntryPoint);
	float tempAngularVelocity = calculateAngularVeloctiyOnCurve(track,currentRadius);
	float delta = abs(tempAngularVelocity - initialAngularVelocity);

	if (d > 0.1)
	{
		
		int sign = mAngularVelocity > 0 ? 1 : -1;
		if (currentRadius < initialRadiusForEntryPoint)
		{ 
			if (abs(mAngularVelocity < abs(initialAngularVelocity) + delta))
			{
				mAngularVelocity = (abs(initialAngularVelocity) - 0.0001 * d) * sign;
			}
			
		}
		else
		{
			if (abs(mAngularVelocity < abs(initialAngularVelocity) + delta))
			{
				mAngularVelocity = (abs(initialAngularVelocity) + 0.0001 * d) * sign;
			}
		}
	}
	else
	{
		mAngularVelocity = initialAngularVelocity;
	}
	
	mRigidBody->setAngularVelocity(btVector3(0, mAngularVelocity, 0));

	btScalar angle = mRigidBody->getWorldTransform().getRotation().getAngle();
}

void 
Car::autoDrive()
{
	
	mCurrentSpeed = mRigidBody->getLinearVelocity().length();
	if(collideWithType != BaseObject::TRACK)
	{
		return;
	}
	Document &document = Configuration::getInstance()->mDocument;

	char* trackListString = level == 0 ? "track_list_0" : "track_list_1";
	int num = getTrackNum(level);

	int angle = -1;


	if (num > -1)
	{
		Value &track = document[trackListString][Configuration::concatenate("track", std::to_string(num).c_str())];
		angle = track["centralAngle"].GetInt();
		angle > 0 ? moveOnCurveTrack(track) : moveOnStraightTrack(track);

		if (oldTrackNum != num)
		{
			mAngularVelocity = 0.f;
			if (angle > 0)
			{
				float initialRadius;
				initialRadiusForEntryPoint = 32 * 6;
				mAngularVelocity = calculateAngularVeloctiyOnCurve(track, initialRadiusForEntryPoint);
			}
			int carRotation = track["carOrietation"].GetInt();
			setRotation(M_PI * carRotation / 180);
			mRigidBody->setAngularVelocity(btVector3(0, mAngularVelocity, 0));
		}
		oldTrackNum = num;
	}	
	
	//when game reset tracknum is -1, so move the car foward a little bit
	btVector3 carPos = mRigidBody->getWorldTransform().getOrigin();

	Ogre::Vector3 direction = mSceneNode->getOrientation() * Ogre::Vector3::UNIT_Z;
	btVector3 velocity = btVector3(direction.x, 0, direction.z).normalize() *  ((collideWithType == BaseObject::TRACK) ? mMaxSpeed : mMaxSpeed * 0.2);
	mRigidBody->activate();
	mRigidBody->setLinearVelocity(velocity);
}

bool
Car::checkCurve(Value &track, float scale, float width, int angle)
{
	float radius = scale * track["radius"].GetDouble();
	Value &trackPosArray = track["postion"];
	float x = scale * trackPosArray[SizeType(0)].GetDouble();
	float y = scale * trackPosArray[SizeType(1)].GetDouble();
	float z = scale * trackPosArray[SizeType(2)].GetDouble();

	Value &startPosArray = track["start"];
	float startX = scale * startPosArray[SizeType(0)].GetDouble();
	float startY = scale * startPosArray[SizeType(1)].GetDouble();
	float startZ = scale * startPosArray[SizeType(2)].GetDouble();

	Value &endPosArray = track["end"];
	float endX = scale * endPosArray[SizeType(0)].GetDouble();
	float endY = scale * endPosArray[SizeType(1)].GetDouble();
	float endZ = scale * endPosArray[SizeType(2)].GetDouble();

	btVector3 carPos= mRigidBody->getWorldTransform().getOrigin();
	float distance = getDistance(btVector3(carPos.getX(),y,carPos.getZ()), btVector3(x,y,z));

	float xMin, xMax, zMin, zMax;
	xMin = zMin = std::numeric_limits<float>::infinity();
	xMax = zMax = -std::numeric_limits<float>::infinity();
	

	//quater circle
	if ((angle / 90) % 2 == 1)
	{
		xMax = getLargerValue(startX, endX);
		xMin = getSmallerValue(startX, endX);
		zMax = getLargerValue(startZ, endZ);
		zMin = getSmallerValue(startZ, endZ);

		//fix with width
		if (x < xMax)
		{
			xMax = xMax + width;
		}
		if (x > xMin)
		{
			xMin = xMin - width;
		}
		if (z < zMax)
		{
			zMax = zMax + width;
		}
		if (z > zMin)
		{
			zMin = zMin - width;
		}
	}

	//semi circle
	else
	{
		Value &startPosArray = track["mid"];
		float midX = scale * startPosArray[SizeType(0)].GetDouble();
		float midY = scale * startPosArray[SizeType(1)].GetDouble();
		float midZ = scale * startPosArray[SizeType(2)].GetDouble();

		xMax = getLargerValue(startX, endX);
		xMin = getSmallerValue(startX, endX);
		zMax = getLargerValue(startZ, endZ);
		zMin = getSmallerValue(startZ, endZ);

		xMax = getLargerValue(xMax, midX);
		xMin = getSmallerValue(xMin, midX);
		zMax = getLargerValue(zMax, midZ);
		zMin = getSmallerValue(zMin, midZ);

		//fix with width
		//startx == endx
		if (abs(startX - endX) < 0.0001f)
		{
			//fix with width
			if (x < xMax)
			{
				xMax = xMax + width;
			}
			if (x > xMin)
			{
				xMin = xMin - width;
			}
			zMax = zMax + width;
			zMin = zMin - width;
	
		}
		//startx != endx
		else
		{
			//fix with width
			xMax = xMax + width;
			xMin = xMin - width;

			if (z < zMax)
			{
				zMax = zMax + width;
			}
			if (z > zMin)
			{
				zMin = zMin - width;
			}
		}
	}

	if (distance < radius + width && distance > radius - width &&
		carPos.getX() < xMax && carPos.getX() > xMin && 
		carPos.getZ() < zMax && carPos.getZ() > zMin)
	{
		return true;
	}
	return false;
}

bool 
Car::checkStraight(Value &track, float scale, float width, int angle)
{
	float length = track["length"].GetDouble();
	Value &trackPosArray = track["postion"];
	float x = scale * trackPosArray[SizeType(0)].GetDouble();
	float y = scale * trackPosArray[SizeType(1)].GetDouble();
	float z = scale * trackPosArray[SizeType(2)].GetDouble();

	Value &startPosArray = track["start"];
	float startX = scale * startPosArray[SizeType(0)].GetDouble();
	float startY = scale * startPosArray[SizeType(1)].GetDouble();
	float startZ = scale * startPosArray[SizeType(2)].GetDouble();

	Value &endPosArray = track["end"];
	float endX = scale * endPosArray[SizeType(0)].GetDouble();
	float endY = scale * endPosArray[SizeType(1)].GetDouble();
	float endZ = scale * endPosArray[SizeType(2)].GetDouble();

	float xMin, xMax, zMin, zMax;
	xMin = zMin = std::numeric_limits<float>::infinity();
	xMax = zMax = -std::numeric_limits<float>::infinity();

	if (abs(startX - endX) < 0.0001f)
	{
		xMax = x + width;
		xMin = x - width;
		zMax = getLargerValue(startZ, endZ);
		zMin = getSmallerValue(startZ, endZ);
	}
	else
	{
		zMax = z + width;
		zMin = z - width;
		xMax = getLargerValue(startX, endX);
		xMin = getSmallerValue(startX, endX);
	}

	btVector3 carPos= mRigidBody->getWorldTransform().getOrigin();

	if (carPos.getX() < xMax && carPos.getX() > xMin && 
		carPos.getZ() < zMax && carPos.getZ() > zMin)
	{
		return true;
	}
	return false;


}

int
Car::getTrackNum(int level) 
{
	Document &document = Configuration::getInstance()->mDocument;
	int num = -1;
	//0 level1, 1 level2
	char* trackListString = level == 0 ? "track_list_0" : "track_list_1";
	int tracksTotalNum = level == 0 ? 4 : 6;

	for(int i = 0; i < tracksTotalNum; i++) 
	{
		Value &track = document[trackListString][Configuration::concatenate("track", std::to_string(i).c_str())];
		float scale = track["scale"].GetDouble();
		float width = scale * track["width"].GetDouble();
		int angle = track["centralAngle"].GetInt();
		bool isOnTrack = angle > 0? checkCurve(track, scale, width, angle) : checkStraight(track, scale, width, angle);
		
		if (isOnTrack)
		{
			num = i;
			break;
		}
	}

	Ogre::LogManager::getSingleton().logMessage("tracknum " + Ogre::StringConverter::toString(num));
	return num;
}


void 
Car::reset()
{
	mCurrentLap = -1;
	mLeft = true;
	Document &document = Configuration::getInstance()->mDocument;
	Value &car = document["car_list"][Configuration::concatenate("car", std::to_string(mID).c_str())];
	mMaxSpeed = car["maxSpeed"].GetInt();
	mCurrentSpeed = 0.0f;
	mForward = true;
	mRigidBody->setLinearVelocity(btVector3(0,0,0));
	mRigidBody->setAngularVelocity(btVector3(0,0,0));
	collideWithType = BaseObject::TRACK;
	mAngularVelocity = 0;
	carId = mID;
	lapCollision = false;
	initialRadiusForEntryPoint = 0.0f;
	oldTrackNum = - 1;
	mWinner = false;
}

