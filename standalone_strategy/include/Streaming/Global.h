#pragma once

#include "Framework/Settings.h"
#include <iostream>

class Global
{
private:
  inline static Settings theSettings;

public:
  static Settings& getSettings()
  {
    return theSettings;
  }

  static bool settingsExist()
  {
    return true;
  }

  static bool debugOutExists()
  {
    return false;
  }
};
