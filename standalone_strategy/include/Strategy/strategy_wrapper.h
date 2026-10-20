#pragma once

#include "Strategy/StrategyBehaviorControl.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/BehaviorControl/AgentStates.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include <string>
#include <memory>

namespace BHumanStrategy
{
  /**
   * Structure containing common input states that callers can update.
   */
  struct StrategyInputs
  {
    GameState::State gameState = GameState::playing;
    GameState::PlayerState playerState = GameState::active;
    int playerNumber = 1;
    bool isGoalkeeper = false;

    // Robot pose on the field
    Pose2f robotPose = Pose2f(0.f, 0.f, 0.f);

    // Ball on the field
    Vector2f ballPosition = Vector2f(1500.f, 200.f);
    Vector2f ballVelocity = Vector2f::Zero();
    unsigned timeWhenBallLastSeen = 0;
    bool ballSeen = true;
  };

  /**
   * Structure containing outputs resulting from a strategy tick.
   */
  struct StrategyOutputs
  {
    StrategyStatus strategyStatus;
    SkillRequest skillRequest;
    MotionRequest motionRequest;
    AgentStates agentStates;

    std::string roleName;
    std::string positionName;
    std::string tacticName;
    std::string skillName;
    std::string motionName;
  };

  /**
   * Clean public C++ API wrapper class for the decoupled B-Human strategy engine.
   */
  class StrategyEngine
  {
  public:
    explicit StrategyEngine(const std::string& configDir = "config",
                            Strategy::Type strategyType = Strategy::s5v5);
    ~StrategyEngine();

    StrategyEngine(const StrategyEngine&) = delete;
    StrategyEngine& operator=(const StrategyEngine&) = delete;

    /**
     * Executes one tick of the strategy engine.
     */
    StrategyOutputs step(const StrategyInputs& inputs);

    /**
     * Direct reference to the internal StrategyBehaviorControl module.
     */
    StrategyBehaviorControl& getModule();

  private:
    class Impl;
    std::unique_ptr<Impl> impl;
  };
}
