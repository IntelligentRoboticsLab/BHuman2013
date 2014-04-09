option(Scan)
{

  /** Simply sets the necessary angles */
  initial_state(scan)
  {
    transition
	{
	if(state_time > 2000 && !theHeadJointRequest.moving)
          goto lookLeft;
	}
    action
    {
	      SetHeadPanTilt(fromDegrees(0.f), -0.33f, fromDegrees(50.f));
     }
  }

  state(lookLeft)
   {
	transition
	{
if(state_time > 2000 && !theHeadJointRequest.moving)
		goto leftReached;
	}
	action 
	{
	      SetHeadPanTilt(fromDegrees(75.f), -0.33f, fromDegrees(50.f));
	}
   }

   state(lookRight)
{
transition
{
if(state_time > 2000 && !theHeadJointRequest.moving)
		goto rightReached;
}
action
{
	      SetHeadPanTilt(fromDegrees(-75.f), -0.33f, fromDegrees(50.f));
}
}

state(rightReached)
   {
	transition
	{
if(state_time > 2000 && !theHeadJointRequest.moving)
		goto lookLeft;
	}
	action 
	{
	      SetHeadPanTilt(fromDegrees(0.f), -0.33f, fromDegrees(50.f));
	}
   }
state(leftReached)
   {
	transition
	{
if(state_time > 2000 && !theHeadJointRequest.moving)
		goto lookRight;
	}
	action 
	{
	      SetHeadPanTilt(fromDegrees(0.f), -0.33f, fromDegrees(50.f));
	}
   }

}

