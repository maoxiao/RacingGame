#ifndef __BaseObject_h_
#define __BaseObject_h_

#include "btBulletDynamicsCommon.h"
#include "OgreMatrix3.h"
#include "OgreVector3.h"

class BaseObject
{

public:
	enum Type
	{
		WALL,
		TRACK,
		GROUND,
		OBSTACLE,
		POWERUP,
		PROJECTILE1,
		PROJECTILE2,
		CAR
	};
	BaseObject(Ogre::SceneManager *sceneManager);
	BaseObject(Ogre::SceneManager *sceneManager, std::string meshName, Ogre::Vector3 scale, int type, float mass, std::string objectName, int id=-1);
	~BaseObject(void);

	void setIsCollided(bool value) { isCollided = value; };
	bool getIsCollided() { return isCollided; };

	void setcollideWithType(int value) { collideWithType = value; };
	int getcollideWithType() { return collideWithType; };


	//void setType(int value) { mType = value; };
	int getType() { return mType; };
	int getID() { return mID; };
	
	void setRotation(btScalar radian);
	void setPosition(const btVector3& position);
	btVector3 getPosition();
	Ogre::SceneNode * BaseObject::getSceneNode() { return mSceneNode; };
	btRigidBody* getRigidBody() { return mRigidBody; };
	void updatePositionAndOrientation();
	void resetBasicObject();

protected:
	bool isCollided;
	int collideWithType;
	int mType;
	Ogre::SceneManager *mSceneManager;
	Ogre::SceneNode *mSceneNode;
	btRigidBody * mRigidBody;
	std::string mName;
	int mID;

	btTransform mInitialTrans;

	

};

#endif
