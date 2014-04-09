option(HeadControl)
{
  common_transition
  {
    if(!theGroundContactState.contact && theGameInfo.state != STATE_INITIAL)
      goto off;

    switch(theHeadControlMode)
    {
      case HeadControl::off: goto off;
      case HeadControl::lookForward: goto lookForward;
      case HeadControl::scan: goto scan;
      default: goto none;
    }
  }

  initial_state(none) {}
  state(off) {action SetHeadPanTilt(JointData::off, JointData::off, 0.f);}
  state(lookForward) {action LookForward();}
  state(scan) {action Scan();}
}

struct HeadControl
{
  ENUM(Mode,
    none,
    off,
    lookForward,
    scan
  );
};

HeadControl::Mode theHeadControlMode; /**< The head control mode executed by the option HeadControl. */
