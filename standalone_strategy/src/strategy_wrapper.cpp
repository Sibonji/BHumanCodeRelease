#include "Strategy/strategy_wrapper.h"
#include "Framework/Blackboard.h"
#include "Platform/File.h"
#include "Streaming/TypeRegistry.h"
#include "Streaming/FunctionList.h"
#include "Representations/Modeling/ObstacleModel.h"
#include <iostream>

namespace BHumanStrategy
{
  class StrategyEngine::Impl
  {
  public:
    std::unique_ptr<StrategyBehaviorControl> module;

    Impl(const std::string& configDir, Strategy::Type strategyType)
    {
      // Execute global reflection & type registrations
      FunctionList::execute();

      // Configure file search paths
      File::setSearchPath({
        configDir + "/",
        configDir + "/Scenarios/Default/",
        configDir + "/Locations/Default/",
        configDir + "/Behavior/",
        "./config/",
        "./config/Scenarios/Default/",
        "./config/Locations/Default/",
        "standalone_strategy/config/",
        "standalone_strategy/config/Scenarios/Default/",
        "standalone_strategy/config/Locations/Default/",
        "standalone_strategy/config/Behavior/",
        "./"
      });

      // Initialize representations in the Blackboard
      auto& theGameState = Blackboard::getInstance().alloc<GameState>("GameState");
      auto& theExtendedGameState = Blackboard::getInstance().alloc<ExtendedGameState>("ExtendedGameState");
      auto& theRobotPose = Blackboard::getInstance().alloc<RobotPose>("RobotPose");
      auto& theFieldBall = Blackboard::getInstance().alloc<FieldBall>("FieldBall");
      auto& theBallModel = Blackboard::getInstance().alloc<BallModel>("BallModel");
      auto& theTeamBallModel = Blackboard::getInstance().alloc<TeamBallModel>("TeamBallModel");
      auto& theFrameInfo = Blackboard::getInstance().alloc<FrameInfo>("FrameInfo");
      auto& theFieldDimensions = Blackboard::getInstance().alloc<FieldDimensions>("FieldDimensions");
      auto& theBallSpecification = Blackboard::getInstance().alloc<BallSpecification>("BallSpecification");
      auto& theBehaviorParameters = Blackboard::getInstance().alloc<BehaviorParameters>("BehaviorParameters");
      auto& theWalkingEngineOutput = Blackboard::getInstance().alloc<WalkingEngineOutput>("WalkingEngineOutput");
      auto& theSetupPoses = Blackboard::getInstance().alloc<SetupPoses>("SetupPoses");
      auto& theFallDownState = Blackboard::getInstance().alloc<FallDownState>("FallDownState");
      auto& theMotionInfo = Blackboard::getInstance().alloc<MotionInfo>("MotionInfo");
      auto& theIndirectKick = Blackboard::getInstance().alloc<IndirectKick>("IndirectKick");
      auto& theBallSearchParticles = Blackboard::getInstance().alloc<BallSearchParticles>("BallSearchParticles");
      auto& theFieldInterceptBall = Blackboard::getInstance().alloc<FieldInterceptBall>("FieldInterceptBall");
      auto& theSentTeamMessage = Blackboard::getInstance().alloc<SentTeamMessage>("SentTeamMessage");
      auto& theReceivedTeamMessages = Blackboard::getInstance().alloc<ReceivedTeamMessages>("ReceivedTeamMessages");
      Blackboard::getInstance().alloc<SkillRequest>("SkillRequest");
      Blackboard::getInstance().alloc<StrategyStatus>("StrategyStatus");
      Blackboard::getInstance().alloc<AgentStates>("AgentStates");
      auto& theExpectedGoals = Blackboard::getInstance().alloc<ExpectedGoals>("ExpectedGoals");
      auto& thePassEvaluation = Blackboard::getInstance().alloc<PassEvaluation>("PassEvaluation");
      auto& thePassTargetEvaluation = Blackboard::getInstance().alloc<PassTargetEvaluation>("PassTargetEvaluation");
      auto& theKickAtGoalEvaluation = Blackboard::getInstance().alloc<KickAtGoalEvaluation>("KickAtGoalEvaluation");
      auto& theLibPosition = Blackboard::getInstance().alloc<LibPosition>("LibPosition");
      Blackboard::getInstance().alloc<GlobalOpponentsModel>("GlobalOpponentsModel");
      Blackboard::getInstance().alloc<GlobalTeammatesModel>("GlobalTeammatesModel");
      Blackboard::getInstance().alloc<ObstacleModel>("ObstacleModel");
      Blackboard::getInstance().alloc<ClearTarget>("ClearTarget");
      Blackboard::getInstance().alloc<DribbleTarget>("DribbleTarget");

      // Load config files
      theFieldDimensions.load();
      {
        InMapFile stream("ballSpecification.cfg");
        if(stream.exists())
          stream >> theBallSpecification;
      }
      {
        InMapFile stream("behaviorParameters.cfg");
        if(stream.exists())
          stream >> theBehaviorParameters;
      }

      // Default walking capabilities
      theWalkingEngineOutput.maxSpeed.translation = Vector2f(300.f, 200.f);
      theWalkingEngineOutput.maxSpeed.rotation = 1.0f;

      // Default fall-down state
      theFallDownState.state = FallDownState::upright;

      // Default evaluation functions
      theExpectedGoals.getRating = [](const Vector2f&, bool, bool, bool) -> float { return 0.5f; };
      theExpectedGoals.getOpponentRating = [](const Vector2f&) -> float { return 0.5f; };
      theExpectedGoals.xG = [](const Vector2f&) -> float { return 0.5f; };
      theExpectedGoals.xGA = [](const Vector2f&) -> float { return 0.5f; };

      thePassEvaluation.getRating = [](const Vector2f&, const Vector2f&, bool) -> float { return 0.5f; };
      thePassTargetEvaluation.getRating = [](const Vector2f&, const Vector2f&, int) -> float { return 0.5f; };
      thePassTargetEvaluation.getBestPassData = [](const Vector2f& b, const Vector2f& t, int tm) -> TargetEvaluation
      {
        TargetEvaluation ev;
        ev.isValid = true;
        ev.ballOnField = b;
        ev.targetOnField = t;
        ev.bestShiftedTargetOnField = t;
        ev.teammate = tm;
        return ev;
      };
      thePassTargetEvaluation.setLastPassTarget = [](const TargetEvaluation&) {};
      theKickAtGoalEvaluation.update = []() {};

      theLibPosition.isNearPost = [](const Pose2f&) -> std::tuple<bool, bool> { return { false, false }; };
      theLibPosition.isInOwnGoalArea = [](const Vector2f&) -> bool { return false; };
      theLibPosition.isNearOwnGoalArea = [](const Vector2f&, float, float) -> bool { return false; };
      theLibPosition.distanceToOwnGoalGreaterThan = [](float) -> bool { return true; };

      theFrameInfo.time = 1000;

      theGameState.phase = GameState::firstHalf;
      theGameState.state = GameState::playing;
      theGameState.playerState = GameState::active;
      theGameState.playerNumber = 1;
      theGameState.gameControllerActive = true;
      theGameState.leftHandTeam = true;
      theGameState.ownTeam.playerStates.fill(GameState::unstiff);
      theGameState.ownTeam.playerStates[0] = GameState::active;
      theGameState.ownTeam.goalkeeperNumber = 1;

      // Instantiate module
      module = std::make_unique<StrategyBehaviorControl>();
      if(strategyType != Strategy::none)
      {
        module->strategy = strategyType;
      }
    }

    StrategyOutputs step(const StrategyInputs& inputs)
    {
      auto& theGameState = Blackboard::getInstance().get<GameState>();
      auto& theExtendedGameState = Blackboard::getInstance().get<ExtendedGameState>();
      auto& theRobotPose = Blackboard::getInstance().get<RobotPose>();
      auto& theFieldBall = Blackboard::getInstance().get<FieldBall>();
      auto& theBallModel = Blackboard::getInstance().get<BallModel>();
      auto& theTeamBallModel = Blackboard::getInstance().get<TeamBallModel>();
      auto& theFrameInfo = Blackboard::getInstance().get<FrameInfo>();

      theFrameInfo.time += 20;

      theGameState.state = inputs.gameState;
      theGameState.playerState = inputs.playerState;
      theGameState.playerNumber = inputs.playerNumber;
      theGameState.ownTeam.playerStates.fill(GameState::unstiff);
      if(inputs.playerNumber >= Settings::lowestValidPlayerNumber &&
         inputs.playerNumber <= Settings::highestValidPlayerNumber)
      {
        theGameState.ownTeam.playerStates[inputs.playerNumber - Settings::lowestValidPlayerNumber] = inputs.playerState;
      }
      theGameState.ownTeam.goalkeeperNumber = inputs.isGoalkeeper ? inputs.playerNumber : 1;

      theRobotPose = inputs.robotPose;

      theBallModel.estimate.position = inputs.robotPose.inverse() * inputs.ballPosition;
      theBallModel.estimate.velocity = inputs.ballVelocity;
      theBallModel.timeWhenLastSeen = inputs.ballSeen ? theFrameInfo.time : inputs.timeWhenBallLastSeen;
      theBallModel.timeWhenDisappeared = theBallModel.timeWhenLastSeen;

      theFieldBall.positionOnField = inputs.ballPosition;
      theFieldBall.positionRelative = theBallModel.estimate.position;
      theFieldBall.endPositionOnField = inputs.ballPosition;
      theFieldBall.endPositionRelative = theBallModel.estimate.position;
      theFieldBall.timeSinceBallWasSeen = inputs.ballSeen ? 0 : theFrameInfo.getTimeSince(inputs.timeWhenBallLastSeen);

      theTeamBallModel.isValid = inputs.ballSeen;
      theTeamBallModel.position = inputs.ballPosition;
      theTeamBallModel.velocity = inputs.ballVelocity;

      SkillRequest skillReq;
      module->update(skillReq);

      StrategyStatus stratStatus;
      module->update(stratStatus);

      AgentStates agentStates;
      module->update(agentStates);

      theExtendedGameState.stateLastFrame = theGameState.state;
      theExtendedGameState.playerStateLastFrame = theGameState.playerState;

      // Translate SkillRequest to MotionRequest
      MotionRequest motionReq;
      switch(skillReq.skill)
      {
        case SkillRequest::stand:
          motionReq.motion = MotionRequest::stand;
          break;
        case SkillRequest::walk:
          motionReq.motion = MotionRequest::walkToPose;
          motionReq.walkTarget = theRobotPose.inverse() * skillReq.target;
          break;
        case SkillRequest::shoot:
        case SkillRequest::pass:
          motionReq.motion = MotionRequest::walkToBallAndKick;
          motionReq.targetDirection = theRobotPose.rotation;
          break;
        case SkillRequest::dribble:
          motionReq.motion = MotionRequest::dribble;
          motionReq.targetDirection = skillReq.target.rotation;
          break;
        case SkillRequest::waitShoot:
        case SkillRequest::observe:
        case SkillRequest::block:
        case SkillRequest::mark:
        case SkillRequest::clear:
        default:
          motionReq.motion = MotionRequest::stand;
          break;
      }

      StrategyOutputs outputs;
      outputs.strategyStatus = stratStatus;
      outputs.skillRequest = skillReq;
      outputs.motionRequest = motionReq;
      outputs.agentStates = agentStates;

      outputs.roleName = TypeRegistry::getEnumName(stratStatus.role);
      outputs.positionName = TypeRegistry::getEnumName(stratStatus.position);
      outputs.tacticName = TypeRegistry::getEnumName(stratStatus.acceptedTactic);
      outputs.skillName = TypeRegistry::getEnumName(skillReq.skill);
      outputs.motionName = TypeRegistry::getEnumName(motionReq.motion);

      return outputs;
    }
  };

  StrategyEngine::StrategyEngine(const std::string& configDir, Strategy::Type strategyType)
    : impl(std::make_unique<Impl>(configDir, strategyType))
  {}

  StrategyEngine::~StrategyEngine() = default;

  StrategyOutputs StrategyEngine::step(const StrategyInputs& inputs)
  {
    return impl->step(inputs);
  }

  StrategyBehaviorControl& StrategyEngine::getModule()
  {
    return *impl->module;
  }
}
