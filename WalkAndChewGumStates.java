// ------------------------------------------------------------------
// Cleaned up and example state machine from GEARSinc
// https://www.youtube.com/watch?v=cPgJ8bbBj_o&t
// ------------------------------------------------------------------

public class StateMachine_Example extends OpMode
{
	// The List of States for Legs
	private enum LegState
	{
		LEG_STATE_INITIAL,
		LEG_STATE_WALK,
		LEG_STATE_TURN,
		LEG_STATE_STOP,
	}
	
	// Leg State variables
	private ElapsedTime LegStateTime = new ElapsedTime(); 	// Time into current state
	private int LegStateDist = 0;                          	// Distance state has walked
	private LegState CurrentLegState;                       // What is the current state
	private boolean InitLegState = FALSE;					// Is this the first time in a state
	
	// Init Leg State
	newLegState(LegState.LEG_STATE_INITIAL);
	
	// The List of States for Mouth
	private enum MouthState
	{
		MOUTH_STATE_INITIAL,
		MOUTH_STATE_INSERT_GUM,
		MOUTH_STATE_CHEW_GUM,
		MOUTH_STATE_SPIT_OUT_GUM,
		MOUTH_STATE_STOP_CHEWING,
	}
	
	// Mouth State variables
	private ElapsedTime MouthStateTime = new ElapsedTime(); // Time into current state
	private int Pieces of GumLeft = 12;						// Amount of gum left
	private MouthState CurrentMouthState;       			// What is the current state
	private boolean InitMouthState = FALSE;					// Is this the first time in a state
	
	// Init Leg State
	newMouthState(MouhtState.Mouth_STATE_INITIAL);
	
	// The Main Loop!!!!
	public void loop()
	{
	
		/* Each STATE's case code does the following
		/  1: Look for EVENT/EVENTS that cause a STATE change
		/  2: if an EVENT is found, take required ACTION, set the next state
		/  3:if no EVENT is found, do processing for state
		*/
	
		// Leg State Machine Switch
		switch (CurrentLegState)
		{
			case LEG_STATE_INITIAL:   // State we are in until we start walking
				if (InitLegState)				// Begin State
				{
					InitLegState = FALSE;
				}
				if (Time to start walking)		// Start walking
				{
					newLegState(LegState.LEG_STATE_WALK);
				}
				else							// Stand still
				{
					stand around
				}
				
				break;
				
			case LEG_STATE_WALK:	// Were walking!
				if (InitLegState)					// Begin walking
				{
					setup legs
					start legs walking forward
					InitLegState = FALSE;
				}
				if (We bumped into something)	// Time to start turning
				{
					stop legs
					newLegState(LegState.LEG_STATE_TURN)
				}
				elseif (We are tired)			// time to stop walking
				{
					stop legs
					newLegState(LegState.LEG_STATE_STOP)
				}
				else							// Keep walking
				{
					legs in walk forward mode
				}
				
				break;
				
			case LEG_STATE_TURN:
				if (InitLegState)					// Begin turning
				{
					setup legs
					start legs turning left
					InitLegState = FALSE;
				}
				if (Nothing in front of us)		// Time to start walking
				{
					stop legs
					newLegState(LegState.LEG_STATE_WALK)
				}
				else							// Keep turning
				{
					legs in turn left mode
				}
				
				break;
				
			case LEG_STATE_STOP:
				stop legs
				lie down
				newMouthState(MouthState.MOUTH_STATE_STOP)
				sleep
				
				break;
		
		}	// End Leg State Machine Switch
	
		// Mouht State Machine Switch
		switch (CurrentMouthState)
		{
			case MOUTH_STATE_INITIAL:   // State we are in until we start chewing
				if (InitMouthState)				// Begin State
				{
					InitMouthState = FALSE;
				}
				if (Time to start chewing)		// Start chewing
				{
					newMouthState(MouthState.MOUTH_STATE_INSERT_GUM);
				}
				else							// Stand still
				{
					stand around
				}
				
				break;
				
			case MOUTH_STATE_INSERT_GUM:	// Looking forward to gum!!
				if (InitMouthState)				// Begin Inserting gum
				{
					setup arms and hands
					start putting gum in mouth
					InitMouthState = FALSE;
				}
				if (gum is in mouth)			// Time to start chewing
				{
					stop inserting gum
					newMouthState(MouthState.MOUTH_STATE_CHEW_GUM)
				}
				else							// Keep inserting gum
				{
					arms and hands, keep putting gum in mouth!
				}
				
				break;
				
			case MOUTH_STATE_CHEW_GUM:
				if (InitMouthState)				// Begin Chewing
				{
					setup mouth
					start mouth chewing
					InitMouthState = FALSE;
				}
				if (Gum flavor gone)			// Time for new gum!
				{
					stop chewing
					newMouthState(MouthState.MOUTH_STATE_SPIT_OUT_GUM)
				}
				else							// Keep chweing
				{
					keep mouth in chew mode
				}
				
			case MOUTH_STATE_SPIT_OUT_GUM:
				if (InitMouthState)				// Begin Spitting
				{
					setup mouth for spitting out
					start spitting gum out
					InitMouthState = FALSE;
				}
				if (time to sleep and gum out of mouth)		// Time to sleep
				{
					stop spitting gum out
					newMouthState(MouthState.MOUTH_STATE_STOP)
				}
				elseif (gum out of mouth)		// Time for more gum
				{
					stop spitting gum out
					newMouthState(MouthState.MOUTH_STATE_INSERT_GUM);
				else							// Keep spitting gum out
				{
					keep spitting mouth out
				}
				
				break;
				
			case MOUTH_STATE_STOP:
				
				break;
		
		}	// End Mouth State Machine Switch
		
	}	// End main loop
	

	private void newLegState(LegState newState)
	{
		// Reset the state time and change to the new state
		LegStateTime.reset();
		CurrentLegState = newState;
		StateDist = 0;
		InitLegState = TRUE;
	}
	
	private void newMouthState(MouhtState newState)
	{
		// Reset the state time and change to the new state
		MouthStateTime.reset();
		CurrentMouthState = newState;
		InitMouthState = TRUE;
	}
	
}
1