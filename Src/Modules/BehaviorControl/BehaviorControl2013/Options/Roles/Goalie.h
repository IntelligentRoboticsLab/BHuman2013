/** Amogh's Goalie */
option(Goalie)
{
  initial_state(start)
  {
    transition
    {
      if((theRobotPose.translation.x > -3900.f || theRobotPose.translation.x < -4500.f || theRobotPose.translation.y < -1100.f || theRobotPose.translation.y > 1100.f))
        goto walktogoal;
	  //If ball in goal area
	  else if(theBallModel.estimate.position.x+theRobotPose.translation.x < -3100.f && theBallModel.estimate.position.y-theRobotPose.translation.y < 1100.f && theBallModel.estimate.position.y-theRobotPose.translation.y > - 1100.f)
		goto walktoballandkick;
    }
    action
    {
      theHeadControlMode = HeadControl::scan;
      Stand();
    }
  }
  state(walktogoal)
  {
	transition
	{
		//If ball in goal area
		if(theBallModel.estimate.position.x+theRobotPose.translation.x < -3100.f && theBallModel.estimate.position.y-theRobotPose.translation.y < 1100.f && theBallModel.estimate.position.y-theRobotPose.translation.y > - 1100.f)
			goto walktoballandkick;
	}
	action
	{
		theHeadControlMode = HeadControl::scan;
		//InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (theFieldDimensions.xPosOwnGroundline - theRobotPose.translation.x),  -theRobotPose.translation.y));
		WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (theFieldDimensions.xPosOwnGroundline - theRobotPose.translation.x),  -theRobotPose.translation.y));
	}
  }
  
  state(walktoballandkick)
  {
	transition
	{
		//If ball outside goal area
		if(!(theBallModel.estimate.position.x+theRobotPose.translation.x < -3100.f && theBallModel.estimate.position.y-theRobotPose.translation.y < 1100.f && theBallModel.estimate.position.y-theRobotPose.translation.y > - 1100.f))
			goto walktogoal;
      if(action_done&&std::abs(libCodeRelease.angleToGoal) < fromDegrees(10.f) && std::abs(theBallModel.estimate.position.y) < 100.f)
        goto alignBehindBall;
	}
	action
	{
		theHeadControlMode = HeadControl::lookForward;
		//InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
		WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
	}
  }

  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto walktogoal;
      if(libCodeRelease.between(theBallModel.estimate.position.y, 20.f, 50.f)
        && libCodeRelease.between(theBallModel.estimate.position.x, 140.f, 170.f)
        && std::abs(libCodeRelease.angleToGoal) < fromDegrees(2.f))
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 150.f, theBallModel.estimate.position.y - 30.f));
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
}
