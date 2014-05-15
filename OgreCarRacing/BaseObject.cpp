

#include "BaseObject.h"

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


BaseObject::BaseObject(Ogre::SceneManager *sceneManager):mSceneManager(sceneManager)
{
	isCollided = false;
	collideWithType = -1;
}

BaseObject::BaseObject(Ogre::SceneManager *sceneManager, std::string meshName, Ogre::Vector3 scale, int type, float mass, std::string objectName, int id):
						mSceneManager(sceneManager),mName(objectName), mType(type), mID(id)
{
	isCollided = false;
	collideWithType = -1;

	Ogre::Entity *entity = mSceneManager->createEntity(mName, meshName);
	mSceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	mSceneNode->attachObject(entity);
	mSceneNode->scale(scale);

	BtOgre::StaticMeshToShapeConverter converter(mSceneManager->getEntity(mName));
	btCollisionShape* shape = converter.createConvex();

	btTransform trans;
	trans.setIdentity();
	shape->setLocalScaling(btVector3(scale.x,scale.y,scale.z));

	mRigidBody = PhysicsWorld::getInstance()->createRigidBody(mass, trans, shape);
	mRigidBody->setUserPointer(this);
	

	mInitialTrans = trans;
}

void
BaseObject::setRotation(btScalar radian)
{
	btTransform trans;
	trans = mRigidBody->getWorldTransform();
	btQuaternion quat;
	quat.setEuler(radian, 0, 0);
	trans.setRotation(quat);
	mRigidBody->setWorldTransform(trans);
	updatePositionAndOrientation();
}

void 
BaseObject::setPosition(const btVector3& position)
{
	btTransform trans = mRigidBody->getWorldTransform();
	trans.setOrigin(position);
	mRigidBody->setWorldTransform(trans);
	updatePositionAndOrientation();
}

btVector3
BaseObject::getPosition()
{
	btTransform trans = mRigidBody->getWorldTransform();
	return trans.getOrigin();
}

void 
BaseObject::updatePositionAndOrientation()
{

	btQuaternion orientation = mRigidBody->getWorldTransform().getRotation();
	btVector3 position = mRigidBody->getWorldTransform().getOrigin();
	mSceneNode->setOrientation(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ());
	mSceneNode->setPosition(Ogre::Vector3(position.getX(), position.getY(), position.getZ()));
	btTransform trans = mRigidBody->getWorldTransform();
}

void
BaseObject::resetBasicObject()
{
	isCollided = false;
	collideWithType = -1;
	mRigidBody->setWorldTransform(mInitialTrans);
	updatePositionAndOrientation();
}

BaseObject::~BaseObject(void)
{
}
