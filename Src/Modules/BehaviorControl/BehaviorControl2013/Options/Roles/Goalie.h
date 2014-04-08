/** The ultimate Keeper without common decision */
option(Goalie)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));

    }
  }

  state(turnToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < fromDegrees(5.f))
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(theBallModel.estimate.position.abs() < 50.f)
        goto alignToGoal;

    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(libCodeRelease.angleToGoal) < fromDegrees(10.f) && std::abs(theBallModel.estimate.position.y) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y, 20.f, 50.f)
        && libCodeRelease.between(theBallModel.estimate.position.x, 140.f, 170.f)
        && std::abs(libCodeRelease.angleToGoal) < fromDegrees(2.f))
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	
    }
  }
  
  state(searchForBall)
  {
    transition
    {
      if(theRobotPose.translation.x < -3900 || theRobotPose.translation.x > -4500 || theRobotPose.translation.y < -1100 || theRobotPose.translation.y > 1100)
      goto goToGoaliePosition;
      if(libCodeRelease.timeSinceBallWasSeen() < 300)
      goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	
    }
  }
  state(goToGoaliePosition)
  {
      transition
      {
	if(theRobotPose.translation.x < -3900 || theRobotPose.translation.x > -4500 || theRobotPose.translation.y < -1100 || theRobotPose.translation.y > 1100)
      		goto goToGoaliePosition;
      	if(libCodeRelease.timeSinceBallWasSeen() < 300)
      		goto turnToBall;
      }
      action
      {
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, -4490, 0));
      //       WalkToTarget(70,Pose2D(libCodeRelease.angleToGoal, -4490 , 0));        
      }
   }
}
