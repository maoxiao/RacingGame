#include "Sound.h"

#include "OgreOggSound.h"

Sound::Sound() {
	mOggSoundManager = OgreOggSound::OgreOggSoundManager::getSingletonPtr();

	Ogre::LogManager::getSingleton().logMessage("start " + Ogre::StringConverter::toString(1));

	mOggSoundManager->init();
	Ogre::LogManager::getSingleton().logMessage("start " + Ogre::StringConverter::toString(2));

	createFireSound();
	Ogre::LogManager::getSingleton().logMessage("start " + Ogre::StringConverter::toString(3));

	createExplosionSound();
	createMoveSound();
	createMissileSound();

}

void Sound::createFireSound() {
	mOggSoundManager->createSound("fire", "fire.wav", false, false, false);
}

void Sound::createExplosionSound() {
	mOggSoundManager->createSound("explosion", "explosion.wav", false, false, false);
}

void Sound::createMoveSound() {
	mOggSoundManager->createSound("move", "engine.wav", false, false, false);
}

void Sound::createMissileSound() {
	mOggSoundManager->createSound("missilefire", "missilefire.wav", false, false, false);
}




void Sound::playFireSound() {
	mOggSoundManager->getSound("fire")->stop();
	mOggSoundManager->getSound("fire")->play();
}

void Sound::playExplosionSound() {
	if (!mOggSoundManager->getSound("explosion")->isPlaying()) { 
		mOggSoundManager->getSound("explosion")->play();
	} 
}

void Sound::playMoveSound() {
	if (!mOggSoundManager->getSound("move")->isPlaying()) {
	    mOggSoundManager->getSound("move")->play();
	}
}

void Sound::playMissileSound(){
	mOggSoundManager->getSound("missilefire")->stop();
	mOggSoundManager->getSound("missilefire")->play();
}


Sound::~Sound() {
	mOggSoundManager->destroyAllSounds();
}
