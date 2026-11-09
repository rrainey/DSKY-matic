#pragma once

#include <AL/al.h>

#include <string>

namespace dso {

class Sound {
 public:
  Sound(ALuint source);
  virtual ~Sound(void);

  /**
   * Load a sound file.  Do not call dirtectly -- use SoundManager::LoadSound()
   * instead.
   * @param strPath pathname to the file to be loaded
   * @return Returns true on success
   * @see SoundManager::LoadSound()
   */
  bool Load(const std::string& strPath);

  /**
   * Play the sound
   * @param bLoop 'true' to continuously loop the sound
   */
  void Play(bool bLoop);

  /**
   * Stop the sound (if playing)
   */
  void Stop();

 protected:
  /**
   * Buffer holding sound data
   */
  ALuint m_buffer;

  /**
   * location where sounds is emanating
   */
  ALuint m_source;

  ALboolean LoadALData(const std::string& strPath);
};

};  // namespace dso
