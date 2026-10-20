#pragma once

#include "Network/RoboCupGameControlData.h"
#include "Streaming/Enum.h"
#include "Streaming/AutoStreamable.h"
#include <string>
#include <vector>

STREAMABLE(Settings,
{
public:
  ENUM(TeamColor,
  {,
    blue,
    red,
    yellow,
    black,
    white,
    green,
    orange,
    purple,
    brown,
    gray,
  });

  ENUM(RobotType,
  {,
    t1,
    k1,
  });

  static constexpr int lowestValidPlayerNumber = 1;
  static constexpr int highestValidPlayerNumber = MAX_NUM_PLAYERS;

  static int getPortForTeam(int teamNumber) { return 10000 + teamNumber; }

  std::vector<std::string> searchPath;
  std::string robotName = "Default";
  float motionCycleTime = 0.012f;
  RobotType robotType = t1,

  (int)(1) teamNumber,
  (TeamColor)(blue) fieldPlayerColor,
  (TeamColor)(blue) goalkeeperColor,
  (int)(1) playerNumber,
  (std::string)("Default") location,
  (std::string)("Default") scenario,
  (unsigned char)(0) magicNumber,
});



