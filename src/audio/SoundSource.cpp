#include "SoundSource.h"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <stdlib.h>

#include "Sound.h"
#include "SoundManager.h"

using namespace dso;

// Velocity of the source sound.
static ALfloat SourceVel[] = {0.0f, 0.0f, 0.1f};

SoundSource::SoundSource(ALfloat x, ALfloat y, ALfloat z) {
  ALfloat sourcePos[3];
  sourcePos[0] = x;
  sourcePos[1] = y;
  sourcePos[2] = z;
  alGenSources(1, &m_source);
  alSourcef(m_source, AL_PITCH, 1.0f);
  alSourcef(m_source, AL_GAIN, 1.0f);
  alSourcefv(m_source, AL_POSITION, sourcePos);
  alSourcefv(m_source, AL_VELOCITY, SourceVel);
}

SoundSource::~SoundSource(void) {
  int queued;

  alGetSourcei(m_source, AL_BUFFERS_QUEUED, &queued);
  while (queued--) {
    ALuint buffer;
    alSourceUnqueueBuffers(m_source, 1, &buffer);
  }

  alDeleteSources(1, &m_source);
}

dso::Sound *SoundSource::CreateSound(void) { return new dso::Sound(m_source); }

dso::Sound *SoundSource::LoadSound(const std::string &strPath) {
  Sound *pResult = NULL;

  pResult = new Sound(m_source);
  if (!pResult->Load(strPath)) {
    pResult = NULL;
  }

  return pResult;
}