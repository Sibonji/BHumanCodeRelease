#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Communication/GameControllerData.h"
#include <cstring>

void RobotPose::draw() const {}
void RobotPose::verify() const {}
void RobotPose::operator>>(BHumanMessage&) const {}
void RobotPose::operator<<(const BHumanMessage&) {}
void RobotPose::onRead()
{
  inversePose = *this;
  inversePose.invert();
}
void GroundTruthRobotPose::draw() const {}

short RobotPoseCompact::float2half(float) { return 0; }
float RobotPoseCompact::half2float(short) { return 0.f; }
RobotPoseCompact::RobotPoseCompact(const RobotPose& rp) : robotPose(const_cast<RobotPose&>(rp)) {}
void RobotPoseCompact::onRead() {}

void Whistle::operator>>(BHumanMessage&) const {}
void Whistle::operator<<(const BHumanMessage&) {}

GameControllerData::GameControllerData()
{
  std::memset(static_cast<RoboCup::RoboCupGameControlData*>(this), 0, sizeof(RoboCup::RoboCupGameControlData));
}
void GameControllerData::read(In&) {}
void GameControllerData::write(Out&) const {}
void GameControllerData::reg() {}

void BallModel::draw() const {}
void BallModel::verify() const {}
void GroundTruthBallModel::draw() const {}

void ObstacleModel::draw() const {}
void ObstacleModel::verify() const {}

void TeamBallModel::draw() const {}
void TeamBallModel::verify() const {}

void GlobalOpponentsModel::draw() const {}
void GlobalOpponentsModel::verify() const {}

void GlobalTeammatesModel::verify() const {}

void HeadMotionRequest::verify() const {}
