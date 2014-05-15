#include "OgreCamera.h"
#include "Camera.h"
#include "World.h"
#include "OgreVector3.h"
#include "InputHandler.h"

PongCamera::PongCamera(Ogre::Camera *renderCamera, World *world,  InputHandler *inpuHandler) :
	mRenderCamera(renderCamera), mWorld(world), mInputHandler(inpuHandler)
{
	Ogre::SceneNode *playerCar = mWorld->mPlayerCar->getSceneNode();
	Ogre::Vector3 cPos = playerCar->getPosition();
	Ogre::Quaternion cOrientation = playerCar->getOrientation();
	Ogre::Vector3 currentDirection = cOrientation * Ogre::Vector3::UNIT_Z;
	currentDirection.normalise();
	int cameraOffsetZ = -30;
	int cameraOffsetY = 50;
	mRenderCamera->setPosition(currentDirection * cameraOffsetZ + Ogre::Vector3(0, cameraOffsetY, 0) + cPos);

	mRenderCamera->lookAt(cPos);
	PongCamera::cameraAngle = "Player";
	mRenderCamera->setNearClipDistance(2);
	count = -1;
}

void
PongCamera::Think(float time)
{
	const float CAMERA_SPEED = 30;
	const float RADIANS_PER_SECOND = 0.5;

	Ogre::SceneNode *playerCar = mWorld->mPlayerCar->getSceneNode();
	Ogre::Vector3 cPos = playerCar->getPosition();
	Ogre::Quaternion cOrientation = playerCar->getOrientation();
	if (mInputHandler->IsKeyDown(OIS::KC_H))
		{
			count = count + 1;
			if(count > 7){
				count = 0;
				cameraAngle = "Heli";
			}
			
	} else if (mInputHandler->IsKeyDown(OIS::KC_P))
	{
		count = count + 1;
		if(count > 7){
			count = 0;
			cameraAngle = "Player";
		}
	}
	else if(mInputHandler->IsKeyDown(OIS::KC_M))
		{
			count = count + 1;
		if(count > 7){
			count = 0;
			cameraAngle = "Movie";
		}
	} else if(mInputHandler->IsKeyDown(OIS::KC_R))
	{
		count = count + 1;
		if(count > 7){
			count = 0;
			cameraAngle = "Rear";
		}
	}
	if(PongCamera::cameraAngle.compare("Default") != 0){
		if(PongCamera::cameraAngle.compare("Movie") == 0){
		
			mRenderCamera->setPosition( cPos + (60 * Ogre::Vector3(1,0,1)) + 20);
			mRenderCamera->setOrientation(cOrientation);
			mRenderCamera->lookAt(cPos);

		} else if (PongCamera::cameraAngle.compare("Heli") == 0){
			Ogre::Vector3 currentDirection = cOrientation * Ogre::Vector3::UNIT_Z;
			currentDirection.normalise();
			int cameraOffsetZ = -30;
			int cameraOffsetY = 1000;
			mRenderCamera->setPosition(currentDirection * cameraOffsetZ + Ogre::Vector3(0, cameraOffsetY, 0) + cPos);
				
			mRenderCamera->lookAt(cPos);
			mRenderCamera->setNearClipDistance(2);
		} else if(PongCamera::cameraAngle.compare("Player") == 0){
			
			Ogre::Vector3 currentDirection = cOrientation * Ogre::Vector3::UNIT_Z;
			currentDirection.normalise();
			int cameraOffsetZ = -100;
			int cameraOffsetY = 50;
			mRenderCamera->setPosition(currentDirection * cameraOffsetZ + Ogre::Vector3(0, cameraOffsetY, 0) + cPos);
		
			mRenderCamera->lookAt(cPos + 10);
			mRenderCamera->setNearClipDistance(2);

		} else if(PongCamera::cameraAngle.compare("Rear") == 0){
			
			Ogre::Vector3 currentDirection = cOrientation * Ogre::Vector3::UNIT_Z;
			currentDirection.normalise();
			int cameraOffsetZ = 100;
			int cameraOffsetY = 50;
			mRenderCamera->setPosition( currentDirection * cameraOffsetZ + Ogre::Vector3(0, cameraOffsetY, 0) + cPos);
		
			mRenderCamera->lookAt(cPos + 10);
			mRenderCamera->setNearClipDistance(2);
		}
	}
}
