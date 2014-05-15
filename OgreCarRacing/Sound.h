#ifndef __Sound_h_
#define __Sound_h_

namespace OgreOggSound {
	class OgreOggSoundManager;
}

class Sound {
public:
	Sound();
	~Sound();
	void playFireSound();
	void playExplosionSound();
	void playMoveSound();
	void playMissileSound();

protected:
	void createFireSound();
	void createExplosionSound();
	void createMoveSound();
	void createMissileSound();

	OgreOggSound::OgreOggSoundManager *mOggSoundManager;
};

#endif
