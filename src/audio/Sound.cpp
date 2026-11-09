#include "Sound.h"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <stdlib.h>

using namespace dso;

// Position of the source sound.
static ALfloat SourcePos[] = {0.0f, 0.0f, 0.0f};

// Velocity of the source sound.
static ALfloat SourceVel[] = {0.0f, 0.0f, 0.1f};

Sound::Sound(ALuint source) {
  m_buffer = 0;
  m_source = source;

  // alGenBuffers( 1, &m_buffer);
}

Sound::~Sound(void) {
  Stop();
  int queued;

  alGetSourcei(m_source, AL_BUFFERS_QUEUED, &queued);
  while (queued--) {
    alSourceUnqueueBuffers(m_source, 1, &m_buffer);
  }

  alDeleteBuffers(1, &m_buffer);
  alDeleteSources(1, &m_source);
}

bool Sound::Load(const std::string& strPath) {
  return LoadALData(strPath) == AL_TRUE;
}

/**
 * Play the sound
 * @param bLoop 'true' to continuously loop the sound
 */
void Sound::Play(bool bLoop) {
  int nBuffers = 0;
  int queued;

  /*
  alSourceStop(m_source);
  alGetSourcei( m_source, AL_BUFFERS_QUEUED, &queued );
  while(queued--)
  {
      alSourceUnqueueBuffers(m_source, 1, &m_buffer);
  }
  */
  alSourceRewind(m_source);
  alSourcei(m_source, AL_LOOPING, bLoop ? AL_TRUE : AL_FALSE);
  alSourcePlay(m_source);
}

/**
 * Stop the sound (if playing)
 */
void Sound::Stop() {
  alSourceStop(m_source);
  alSourceRewind(m_source);
}

ALboolean Sound::LoadALData(const std::string& strPath) {
  m_buffer = alutCreateBufferFromFile(strPath.c_str());

  if (m_buffer == AL_NONE) {
    return AL_FALSE;
  }

  // Bind buffer with a source.

  alSourcei(m_source, AL_BUFFER, m_buffer);

  // Do an error check and return.

  if (alGetError() != AL_NO_ERROR) return AL_FALSE;

  return AL_TRUE;
}
