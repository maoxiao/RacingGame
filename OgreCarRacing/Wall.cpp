#include "Wall.h"

#include "Ogre.h"
#include "./btogre/include/BtOgreExtras.h"
#include "./btogre/include/BtOgreGP.h"
#include "./btogre/include/BtOgreExtras.h"
#include "PhysicsWorld.h"

Wall::Wall(Ogre::SceneManager *sceneManager, float originX, float originY, float orininZ, float scaleX, float scaleY, float scaleZ, int type):BaseObject(sceneManager)
{
	mType = BaseObject::WALL;
	
	//top
	if(type == 1) {
	Ogre::Plane plane1(Ogre::Vector3::UNIT_Z, Ogre::Vector3(0, 0, -1000));
    Ogre::MeshManager::getSingleton().createPlane("wall1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane1, 2000, 1000, 20, 10, true, 1, 5, 1, Ogre::Vector3::UNIT_Y);
    Ogre::Entity* entityTopWall = mSceneManager->createEntity("wall1");
	Ogre::SceneNode *topNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	topNode ->attachObject(entityTopWall);
    entityTopWall->setMaterialName("Examples/RustySteel");

	
	}


	//bottom
	else if(type == 2){
	Ogre::Plane plane2(-Ogre::Vector3::UNIT_Z, Ogre::Vector3(0, 0, 1000));
    Ogre::MeshManager::getSingleton().createPlane("wall2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane2, 2000, 1000, 20, 10, true, 1, 5, 1, Ogre::Vector3::UNIT_Y);
    Ogre::Entity* entityBottomWall = mSceneManager->createEntity("wall2");
	Ogre::SceneNode *bottomNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	bottomNode ->attachObject(entityBottomWall);
    entityBottomWall->setMaterialName("Examples/RustySteel");

	
	}

	//left wall
	else if(type == 3) {
	Ogre::Plane plane3(-Ogre::Vector3::UNIT_X, Ogre::Vector3(1000, 0, 0));
	Ogre::MeshManager::getSingleton().createPlane("wall3", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane3, 1000, 2000, 20, 10, true, 1, 5, 1, Ogre::Vector3::UNIT_Z);
	Ogre::Entity* entityLeftWall = mSceneManager->createEntity("wall3");
	Ogre::SceneNode *leftNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	leftNode ->attachObject(entityLeftWall);
	entityLeftWall->setMaterialName("Examples/RustySteel");	
	
	
	}


	//right wall
	else if(type == 4) {
	Ogre::Plane plane4(Ogre::Vector3::UNIT_X, Ogre::Vector3(-1000, 0, 0));
    Ogre::MeshManager::getSingleton().createPlane("wall4", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane4, 1000, 2000, 20, 10, true, 1, 5, 1, Ogre::Vector3::UNIT_Z);
    Ogre::Entity* entityRightWall = mSceneManager->createEntity("wall4");
	Ogre::SceneNode *rightNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	rightNode ->attachObject(entityRightWall);
	entityRightWall->setMaterialName("Examples/RustySteel");

	
	}

	btCollisionShape* shape = new btBoxShape(btVector3(btScalar(scaleX), btScalar(scaleY), btScalar(scaleZ)));
	btTransform wallTransform;
	wallTransform.setIdentity();
	wallTransform.setOrigin(btVector3(originX, originY, orininZ));
	mRigidBody = PhysicsWorld::getInstance()->createRigidBody(0.0f, wallTransform, shape);
	mRigidBody->setUserPointer(this);


}


Wall::~Wall(void)
{
}
