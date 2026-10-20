#include "Strategy/strategy_wrapper.h"
#include <iostream>
#include <iomanip>
#include <sys/stat.h>

int main(int argc, char* argv[])
{
  std::cout << "========================================================\n";
  std::cout << "  B-Human Standalone Strategy Engine Verification Test\n";
  std::cout << "========================================================\n\n";

  std::string configPath = "config";
  if(argc > 1)
    configPath = argv[1];
  else
  {
    struct stat st;
    if(stat("standalone_strategy/config", &st) == 0 && S_ISDIR(st.st_mode))
      configPath = "standalone_strategy/config";
  }

  std::cout << "[INIT] Initializing StrategyEngine with config dir: " << configPath << " ...\n";
  BHumanStrategy::StrategyEngine engine(configPath, Strategy::s5v5);
  std::cout << "[INIT] Engine successfully initialized.\n\n";

  // Populate dummy input states as requested in prompt:
  // e.g., GameInfo::state = PLAYING, RobotPose at (0, 0, 0), FieldBall at (1500, 200)
  BHumanStrategy::StrategyInputs inputs;
  inputs.gameState = GameState::playing;
  inputs.playerState = GameState::active;
  inputs.playerNumber = 1;
  inputs.isGoalkeeper = false;
  inputs.robotPose = Pose2f(0.f, 0.f, 0.f);
  inputs.ballPosition = Vector2f(1500.f, 200.f);
  inputs.ballVelocity = Vector2f(0.f, 0.f);
  inputs.ballSeen = true;

  std::cout << "[INPUTS] Playing state, Player #1 (Field Player)\n";
  std::cout << "         Robot Pose: (" << inputs.robotPose.translation.x() << ", "
            << inputs.robotPose.translation.y() << ", rot="
            << inputs.robotPose.rotation << " rad)\n";
  std::cout << "         Ball Pos:   (" << inputs.ballPosition.x() << ", "
            << inputs.ballPosition.y() << ")\n\n";

  std::cout << "--- Executing 5 Ticks of Strategy Update Loop ---\n\n";

  for(int tick = 1; tick <= 5; ++tick)
  {
    // Slightly move robot towards ball each tick to simulate dynamic execution
    inputs.robotPose.translation.x() += 50.f;

    auto outputs = engine.step(inputs);

    std::cout << "Tick #" << tick << ":\n";
    std::cout << "  Tactic:          " << outputs.tacticName << "\n";
    std::cout << "  Position:        " << outputs.positionName << "\n";
    std::cout << "  Role Assignment: " << outputs.roleName << "\n";
    std::cout << "  SkillRequest:    " << outputs.skillName
              << " -> target=(" << outputs.skillRequest.target.translation.x()
              << ", " << outputs.skillRequest.target.translation.y()
              << ", " << outputs.skillRequest.target.rotation << " rad)\n";
    std::cout << "  MotionRequest:   " << outputs.motionName
              << " -> walkTarget=(" << outputs.motionRequest.walkTarget.translation.x()
              << ", " << outputs.motionRequest.walkTarget.translation.y() << ")\n";
    std::cout << "  Active Agents:   " << outputs.agentStates.agents.size() << "\n";
    std::cout << "--------------------------------------------------------\n";
  }

  std::cout << "\n[SUCCESS] Decoupled Strategy Engine executed 5 ticks cleanly.\n";
  return 0;
}
