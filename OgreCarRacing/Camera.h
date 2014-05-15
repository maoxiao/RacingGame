#ifndef __Camera_h_
#define __Camera_h_
#include <string>

class InputHandler;
namespace Ogre
{
    class Camera;
    class Vector3;
}
class World;


class PongCamera
{

public:
	PongCamera(Ogre::Camera *renderCamera, World *world, InputHandler *inpuHandler); 
    void Think(float time);
    // If you have a different cameras, you'll want some acessor methods here.
    //  If your camera always stays still, you could remove this class entirely

protected:
	std::string cameraAngle;
    Ogre::Camera *mRenderCamera; 
    World *mWorld;
	InputHandler *mInputHandler;
	int count;
};

#endif