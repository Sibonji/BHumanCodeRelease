/**
 * @file FieldBall.cpp
 *
 * Declaration of a representation that contains additional information
 * about the ball that is required by the behavior.
 *
 * @author Tim Laue
 */

#include "FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Blackboard.h"
#include "Tools/Modeling/BallPhysics.h"

bool FieldBall::ballWasSeen(int timeInterval) const
{
  return timeSinceBallWasSeen <= timeInterval;
}

Vector2f FieldBall::recentBallPositionOnField(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeamBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamPositionOnField;
  else
    return positionOnField;
}

Vector2f FieldBall::recentBallPositionRelative(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeamBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamPositionRelative;
  else
    return positionRelative;
}

Vector2f FieldBall::recentBallEndPositionOnField(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeamBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamEndPositionOnField;
  else
    return endPositionOnField;
}

Vector2f FieldBall::recentBallEndPositionRelative(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeamBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamEndPositionRelative;
  else
    return endPositionRelative;
}

Vector2f FieldBall::recentBallPropagatedPositionOnField(const float t, const BallSpecification& ballSpecs, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  const bool useTeamEstimate = useTeamBall(ballSeenTimeout, ballDisappearedTimeout);

  const Vector2f& ballPosition = useTeamEstimate ? teamPositionOnField : positionOnField;
  const Vector2f ballVelocity = useTeamEstimate ? teamVelocityOnField : velocityOnField;
  const Vector2f propagatedBallPosition = BallPhysics::propagateBallPosition(ballPosition, ballVelocity, t, ballSpecs);
  return propagatedBallPosition;
}

Vector2f FieldBall::recentBallPropagatedPositionRelative(const float t, const BallSpecification& ballSpecs, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  const bool useTeamEstimate = useTeamBall(ballSeenTimeout, ballDisappearedTimeout);

  const Vector2f& ballPosition = useTeamEstimate ? teamPositionRelative : positionRelative;
  const Vector2f ballVelocity = useTeamEstimate ? teamVelocityRelative : velocityRelative;
  const Vector2f propagatedBallPosition = BallPhysics::propagateBallPosition(ballPosition, ballVelocity, t, ballSpecs);
  return propagatedBallPosition;
}

void FieldBall::recentBallPositions(Vector2f& ballPositionOnField, Vector2f& ballPositionRelative, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeamBall(ballSeenTimeout, ballDisappearedTimeout))
  {
    ballPositionOnField = teamPositionOnField;
    ballPositionRelative = teamPositionRelative;
  }
  else
  {
    ballPositionOnField = positionOnField;
    ballPositionRelative = positionRelative;
  }
}

void FieldBall::recentBallEndPositions(Vector2f& ballEndPositionOnField, Vector2f& ballEndPositionRelative, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && teamBallIsValid && teamBallNewerThanOwnBall)
  {
    ballEndPositionOnField = teamEndPositionOnField;
    ballEndPositionRelative = teamEndPositionRelative;
  }
  else
  {
    ballEndPositionOnField = endPositionOnField;
    ballEndPositionRelative = endPositionRelative;
  }
}

bool FieldBall::useTeamBall(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  return (timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
         && teamBallIsValid && teamBallNewerThanOwnBall;
}

void FieldBall::draw() const {}
