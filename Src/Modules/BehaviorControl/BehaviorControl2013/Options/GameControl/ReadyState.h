/** behavior for the ready state */
option(ReadyState)
{
  initial_state(goToPosition)
  {
    transition
    {
      if(theRobotInfo.number == 1)
	goto goToPositionGoalie;
      else if(theRobotInfo.number == 2)
	goto goToPositionStriker2;
      else if(theRobotInfo.number == 3)
	goto goToPositionStriker3;
      else if(theRobotInfo.number == 4)
	goto goToPositionStriker4;
      else if(theRobotInfo.number == 5)
	goto goToPositionStriker5;
      else if(theRobotInfo.number == 6) // this is the spare robot
	goto goToPositionStriker6;
    }
    action
    {
		theHeadControlMode = HeadControl::scan;
		Stand();
    }
  }

  state(goToPositionGoalie)
  {
    transition
    {
      if(action_done || action_aborted) 
		goto stand;
    }
    action
    {
		//Goalie Position = -4300, 0 
		theHeadControlMode = HeadControl::scan;
		//WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (-4300.f - theRobotPose.translation.x), 0.f - theRobotPose.translation.y));
		InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (-4300.f - theRobotPose.translation.x), 0.f - theRobotPose.translation.y));
	}
  }

  state(goToPositionStriker2)
  {
    transition
    {
      if(action_done || action_aborted) 
		goto stand;
    }
    action
    {
		theHeadControlMode = HeadControl::scan;
		//WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (-1280.f - theRobotPose.translation.x), 1000.f - theRobotPose.translation.y));
		InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (-200.f - theRobotPose.translation.x), 200.f - theRobotPose.translation.y));
	}
  }

  state(goToPositionStriker3)
  {
    transition
    {
      if(action_done || action_aborted) 
		goto stand;
    }
    action
    {
		theHeadControlMode = HeadControl::scan;
		//WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (-200.f - theRobotPose.translation.x), -1500.f - theRobotPose.translation.y));
		InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (-200.f - theRobotPose.translation.x), -350.f - theRobotPose.translation.y));
	}
  }

  state(goToPositionStriker4)
  {
    transition
    {
      if(action_done || action_aborted) 
		goto stand;
    }
    action
    {
		theHeadControlMode = HeadControl::scan;
		//WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (-200.f - theRobotPose.translation.x), 0.f - theRobotPose.translation.y));
		InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (-2000.f - theRobotPose.translation.x), 100.f - theRobotPose.translation.y));
	
	}
  }

  state(goToPositionStriker5)
  {
    transition
    {
      if(action_done || action_aborted) 
		goto stand;
    }
    action
    {
		theHeadControlMode = HeadControl::scan;
		//WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (-400.f - theRobotPose.translation.x), 1000.f - theRobotPose.translation.y));
		InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (-2500.f - theRobotPose.translation.x), 1500.f - theRobotPose.translation.y));
	
	}
  }

  state(goToPositionStriker6)
  {
    transition
    {
      if(action_done || action_aborted) 
		goto stand;
    }
    action
    {
		theHeadControlMode = HeadControl::scan;
		//WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, (-3300.f - theRobotPose.translation.x), -350.f - theRobotPose.translation.y));
		InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, (-2500.f - theRobotPose.translation.x), -1500.f - theRobotPose.translation.y));
	
	}
  }
  state(stand)
  {
    transition
    {
      //if(theMotionInfo.motion == MotionRequest::walk)
	//	goto goToPosition;
    }
    action
    {
      Stand();
    }
  }
}
