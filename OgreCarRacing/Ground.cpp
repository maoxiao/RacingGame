#include "Ground.h"

#include "Ogre.h"
#include "./btogre/include/BtOgreExtras.h"
#include "./btogre/include/BtOgreGP.h"
#include "./btogre/include/BtOgreExtras.h"
#include "PhysicsWorld.h"

Ground::Ground(Ogre::SceneManager *sceneManager):BaseObject(sceneManager)
{
	mType = BaseObject::GROUND;
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);

	Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 2000, 2000, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);

	Ogre::Entity *entGround = mSceneManager->createEntity("GroundEntity", "ground");
	mSceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
	mSceneNode->attachObject(entGround);
	entGround->setMaterialName("Examples/Frost");

	btCollisionShape* shape = new btBoxShape(btVector3(btScalar(1000.), btScalar(0.), btScalar(1000.)));
	btTransform groundTransform;

	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));
	mRigidBody = PhysicsWorld::getInstance()->createRigidBody(0.0f, groundTransform, shape);
	mRigidBody->setUserPointer(this);
}


Ground::~Ground(void)
{
}
