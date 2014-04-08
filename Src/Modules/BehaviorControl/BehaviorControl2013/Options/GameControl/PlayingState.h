
option(PlayingState)
{

  initial_state(play)
  {
    transition
    {   
    }
    action
    {
	if(theRobotInfo.number == 1){
		Goalie();
	}else{
		Striker();
	}
    }
  }
}


