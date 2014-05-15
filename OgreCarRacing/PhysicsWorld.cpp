
#include <stddef.h>

#include "PhysicsWorld.h"


#include "./btogre/include/BtOgreExtras.h"
#include "./btogre/include/BtOgreGP.h"
#include "./btogre/include/BtOgreExtras.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btGeometryUtil.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "Ogre.h"
#include "BaseObject.h"




PhysicsWorld *PhysicsWorld::m_pInstance = NULL;



PhysicsWorld::PhysicsWorld(void)
{
	mCollisionConfiguration = new btDefaultCollisionConfiguration();
	mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);
	mOverlappingPairCache = new btDbvtBroadphase();
	mSolver = new btSequentialImpulseConstraintSolver();
	mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mOverlappingPairCache, mSolver, mCollisionConfiguration);
	mDefaultContactProcessingThreshold = BT_LARGE_FLOAT;
	mDynamicsWorld -> setGravity(btVector3(0,-30,0));
}

PhysicsWorld*
PhysicsWorld::getInstance()
{
	if(m_pInstance==NULL)
		m_pInstance=new PhysicsWorld();
	return m_pInstance;
}


btRigidBody*
PhysicsWorld::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	mCollisionShapes.push_back(shape);

	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);
	cInfo.m_friction = 0.0;

	btRigidBody* body = new btRigidBody(cInfo);

	body->setContactProcessingThreshold(mDefaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);
#endif//

	mDynamicsWorld->addRigidBody(body);

	return body;
}




void
PhysicsWorld::checkCollisions()
{
	
	/* Browse all collision pairs */
	int numManifolds = mDynamicsWorld->getDispatcher()->getNumManifolds();

	for (int i=0; i<numManifolds; i++)
	{
		btPersistentManifold* contactManifold = mDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		//const btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		//const btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		BaseObject *oA = (NULL != obA->getUserPointer()) ? static_cast<BaseObject*>(obA->getUserPointer()) : NULL;
		BaseObject *oB = (NULL != obB->getUserPointer()) ? static_cast<BaseObject*>(obB->getUserPointer()) : NULL;


		if((oA != NULL && BaseObject::TRACK == oA->getcollideWithType()) || (oB != NULL && BaseObject::TRACK == oB->getcollideWithType()))
		{
			
			continue;
		}
		
		/* Check all contacts points */
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			//Ogre::LogManager::getSingleton().logMessage("check j: " + Ogre::StringConverter::toString(j) + "    numContacts: " + Ogre::StringConverter::toString(numContacts));
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			if (pt.getDistance()<0.f)
			{
				
				if (oA->getType() == BaseObject::CAR)
				{
					oA->setcollideWithType(oB->getType());
				}
				if (oB->getType() == BaseObject::CAR)
				{
					oB->setcollideWithType(oA->getType());
				}

				
			}
			
		}
	}

	int numObjects = mDynamicsWorld->getNumCollisionObjects();

}


void
PhysicsWorld::simulate(float time)
{
	mDynamicsWorld -> stepSimulation(time);
	checkCollisions();
}


PhysicsWorld::~PhysicsWorld(void)
{
}

