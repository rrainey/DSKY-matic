#pragma once

#include <AL/al.h>
#include <SoundManager.h>

#include <string>

namespace dso {

class SoundSource {
 public:
  SoundSource(ALfloat x, ALfloat y, ALfloat z);
  virtual ~SoundSource(void);

  dso::Sound *CreateSound(void);

  dso::Sound *LoadSound(const std::string &strPath);

 protected:
  /**
   * location where sounds is emanating
   */
  ALuint m_source;
};

};  // namespace dso
