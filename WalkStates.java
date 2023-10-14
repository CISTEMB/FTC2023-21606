// ------------------------------------------------------------------
// Cleaned up and example state machine from GEARSinc
// https://www.youtube.com/watch?v=cPgJ8bbBj_o&t
// ------------------------------------------------------------------

public class StateMachine_Example extends OpMode
{
	// The List of States
	private enum State
	{
		STATE_INITIAL,
		STATE_WALK,
		STATE_TURN,
		STATE_STOP,
	}
	
	// State variables
	private ElapsedTime StateTime = new ElapsedTime();  // Time into current state
	private int StateDist = 0;                          // Distance state has walked
	private State CurrentState;                         // What is the current state
	private boolean InitState = FALSE;					// Is this the first time in a state
	
	// Init
	newState(State.STATE_INITIAL);
	
	// The Main Loop!!!!
	public void loop()
	{
	
		/* Each STATE's case code does the following
		/  1: If it;s the first time, SETUP the state
		/  2: Look for EVENT/EVENTS that cause a STATE change
		/  3: if an EVENT is found, take required ACTIONS, set the next state
		/  4: if no EVENT is found, do processing for state
		*/
	
		// State Machine Switch
		switch (CurrentState)
		{
			case STATE_INITIAL:   // State we are in until we start walking
				if (InitState)					// Begin State
				{
					InitState = FALSE;
				}
				if (Time to start walking)		// Start walking
				{
					newState(State.State_Walk);
				}
				else							// Stand still
				{
					stand around
				}
				
				break;
				
			case STATE_WALK:	// Were walking!
				if (InitState)					// Begin walking
				{
					setup legs
					start legs walking forward
					InitState = FALSE;
				}
				if (We bumped into something)	// Time to start turning
				{
					stop legs
					newState(State.STATE_TURN)
				}
				elseif (We are tired)			// time to stop walking
				{
					stop legs
					newState(State.STATE_STOP)
				}
				else							// Keep walking
				{
					keep legs in walk forward mode
				}
				
				break;
				
			case STATE_TURN:
				if (InitState)					// Begin turning
				{
					setup legs
					start legs turning left
					InitState = FALSE;
				}
				if (Nothing in front of us)		// Time to start walking
				{
					stop legs
					newState(State.STATE_WALK)
				}
				else							// Keep turning
				{
					legs in turn left mode
				}
				
				break;
				
			case STATE_STOP:
				stop legs
				lie down
				sleep
				
				break;
		
		}	// End State Machine Switch
	
	}	// End main loop
	

	private void newstate(State newState)
	{
		// Reset the state time and change to the new state
		StateTime.reset();
		CurrentState = newState;
		StateDist = 0;
		InitState = TRUE;
	}
	
}
1