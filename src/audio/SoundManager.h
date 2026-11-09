#pragma once

#include <map>
#include <string>

#define SND_COCKPIT_BACKGROUND "cockpit-background"

namespace dso {

class Sound;

typedef std::map<std::string, dso::Sound *> SoundMap;

class SoundManager {
 public:
  SoundManager(void);
  virtual ~SoundManager(void);

  /**
   * Load a sound file.
   * @param strPath pathname to the file to be loaded
   * @return Returns a pointer to the Sound object on success; NULL on error
   */
  dso::Sound *LoadSound(const std::string &strName, const std::string &strPath);

  const char *openAlErrorToString(int err);

 protected:
  SoundMap m_soundMap;
};

extern SoundManager *GetSoundManager();

};  // namespace dso
