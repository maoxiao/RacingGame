#ifndef __PhysicsWorld_h_
#define __PhysicsWorld_h_


#include "btBulletDynamicsCommon.h"

class PhysicsWorld
{
public:
	
	~PhysicsWorld(void);
	static PhysicsWorld * getInstance();

	btRigidBody* createRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);

	void removeObject(btRigidBody* rigidBody);

	void simulate(float time);
	int getSize() { return mCollisionShapes.size(); };


private:

	btDefaultCollisionConfiguration* mCollisionConfiguration;
	btCollisionDispatcher* mDispatcher;
	btBroadphaseInterface* mOverlappingPairCache;
	btSequentialImpulseConstraintSolver* mSolver;
	btDiscreteDynamicsWorld* mDynamicsWorld;
	btAlignedObjectArray<btCollisionShape *> mCollisionShapes;
	btScalar mDefaultContactProcessingThreshold;

	void checkCollisions();


	PhysicsWorld(void);
	static PhysicsWorld * m_pInstance;
	class Garbo//delete instance object
	{
	public:
		~Garbo()
		{
			if(PhysicsWorld::m_pInstance)
			{
				delete PhysicsWorld::m_pInstance;
			}
		}
	};
	static Garbo gb;

};

#endif
