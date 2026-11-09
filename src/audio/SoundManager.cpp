#include "SoundManager.h"

#include "Sound.h"
// #include <alu.h>
#include <AL/alut.h>

#include <cstring>
#include <iostream>
#include <vector>

using namespace dso;
using namespace std;

// Position of the listener.
static ALfloat ListenerPos[] = {0.0f, 0.0f, 1.0f};

// Velocity of the listener.
static ALfloat ListenerVel[] = {0.0f, 0.0f, 0.0f};

// Orientation of the listener. (first 3 elements are "at", second 3 are "up")
static ALfloat ListenerOri[] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f};

static SoundManager *g_pSoundManager = NULL;

dso::SoundManager *dso::GetSoundManager() { return g_pSoundManager; }

SoundManager::SoundManager(void) {
  int argc = 0;
  ALenum result;

  const char *argv[2] = {"dallas", NULL};

  alutInit(&argc, (char **)&argv);
  result = alGetError();
  if (result != AL_NO_ERROR) {
    cout << "openAL error - " << result << "\n\n";
    exit(1);
  }

  // Set listener parameters
  alListenerfv(AL_POSITION, ListenerPos);
  alListenerfv(AL_VELOCITY, ListenerVel);
  alListenerfv(AL_ORIENTATION, ListenerOri);

  ALCdevice *device = alcOpenDevice(NULL);
  ALCcontext *context = alcCreateContext(device, NULL);
  ALCint size;
  alcGetIntegerv(device, ALC_ATTRIBUTES_SIZE, 1, &size);
  std::vector<ALCint> attrs(size);
  alcGetIntegerv(device, ALC_ALL_ATTRIBUTES, size, &attrs[0]);
  for (size_t i = 0; i < attrs.size(); ++i) {
    if (attrs[i] == ALC_MONO_SOURCES) {
      std::cout << "max mono sources: " << attrs[i + 1] << std::endl;
    }

    if (attrs[i] == ALC_STEREO_SOURCES) {
      std::cout << "max stereo sources: " << attrs[i + 1] << std::endl;
    }
  }

  const ALCchar *devicex = alcGetString(NULL, ALC_DEVICE_SPECIFIER),
                *next = devicex + 1;
  size_t len = 0;

  fprintf(stdout, "Devices list:\n");
  fprintf(stdout, "----------\n");
  while (devicex && *devicex != '\0' && next && *next != '\0') {
    fprintf(stdout, "%s\n", device);
    len = strlen(devicex);
    devicex += (len + 1);
    next += (len + 2);
  }
  fprintf(stdout, "----------\n");
  fflush(stdout);

  g_pSoundManager = this;
}

SoundManager::~SoundManager(void) {
  SoundMap::iterator it;

  // Delete all sounds
  for (it = m_soundMap.begin(); it != m_soundMap.end();) {
    delete (*it).second;
    ++it;
  }
  m_soundMap.clear();

  alutExit();
}

const char *SoundManager::openAlErrorToString(int err) {
  switch (err) {
    case AL_NO_ERROR:
      return "AL_NO_ERROR";
    case AL_INVALID_ENUM:
      return "AL_INVALID_ENUM";
    case AL_INVALID_VALUE:
      return "AL_INVALID_VALUE";
    case AL_OUT_OF_MEMORY:
      return "AL_OUT_OF_MEMORY";
    case AL_INVALID_OPERATION:
      return "AL_INVALID_OPERATION";
    /* ... */
    default:
      return "Unknown error code";
  }
}
