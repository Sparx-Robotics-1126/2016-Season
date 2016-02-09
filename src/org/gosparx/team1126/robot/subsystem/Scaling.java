package org.gosparx.team1126.robot.subsystem;
import org.gosparx.team1126.robot.subsystem.Drives;
import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.MagnetSensor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Allows the robot to scale the tower
 * @author Andrew Thompson {andrewt015@gmail.com}
 */
public class Scaling extends GenericSubsystem{
	
	//******************************OBJECTS***********************************
	
	/**
	 * The only instance of Scaling 
	 */
	private static Scaling scaling;
	
	/**
	 * Instance of drives
	 */
	private Drives drives;
	
	/**
	 * Right hook sensor 
	 */
	private MagnetSensor rightHook;
	
	/**
	 * Left hook sensor 
	 */
	private MagnetSensor leftHook;
	
	/**
	 * Solenoid to extend arms to scaling position
	 */
	private Solenoid arms;
	
	/**
	 * Solenoid for the winch ratchet
	 */
	private Solenoid ratchet;
	
	//******************************CONSTANTS***********************************

	/**
	 * Winch in position
	 */
	private final double WINCH_IN_DISTANCE = 16; //FIXME find actual distance
	
	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = false;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;
	
	/**
	 * Value for the solenoid if the ratchet is locked
	 */
	public static final boolean LOCK = true;
	
	/**
	 * Value for the solenoid if the ratchet is unlocked
	 */
	public static final boolean UNLOCK = !LOCK;

	/**
	 * Value of the magnet sensor when not tripped 
	 */
	private static final boolean inverse = false;
	
	//******************************VARIABLES***********************************
	
	/**
	 * The current scaling state
	 */
	private State currentScalingState;
	
	/**
	 * Returns the only instance of scaling
	 */
	public static synchronized Scaling getInstance(){
		if(scaling == null){
			scaling = new Scaling();
		}
		return scaling;
	}
	
	/**
	 * Creates a new scaling 
	 */
	public Scaling() {
		super("Scaling", Thread.NORM_PRIORITY);
	}

	/**
	 * Initializes things 
	 */
	@Override
	protected boolean init() {
		
		//Right 
		rightHook = new MagnetSensor(IO.DIO_HOOK_R, inverse);  //FIXME may not be magnet sensors
		
		//Left
		leftHook = new MagnetSensor(IO.DIO_HOOK_L, inverse);  //FIXME may not actually be magnet sensors
		
		//Other
		drives = Drives.getInstance(); 
		arms = new Solenoid(IO.PNU_CLIMBER_SCALE);
		ratchet = new Solenoid(IO.PNU_WINCH_RATCHET);
		currentScalingState = State.STANDBY;
		setArms(ARMS_DOWN);
		setLock(LOCK);
		
		return true;
	}

	/**
	 * Adds items to the live window, overrides genericSubsystems 
	 */
	@Override
	protected void liveWindow() {
		//FIXME figure out which sensors were using
		String subsystemName = "Scaling";
		LiveWindow.addActuator(subsystemName, "Arms", arms);
		LiveWindow.addActuator(subsystemName, "Ratchet", ratchet);		
	}
	
	/**
	 * Loops
	 */
	@Override
	protected boolean execute() {
		switch(currentScalingState){
		case STANDBY:{
			break;
		}
		case EXTENDING:{
			setArms(ARMS_UP);
				currentScalingState = State.STANDBY;
			}
			break;
		case SCALING: 
			if (rightHook.isTripped() && leftHook.isTripped()){
				setArms(ARMS_DOWN);
				if(drives.isScaleScalingDone())
				{
					LOG.logMessage("Scaling complete");
					currentScalingState = State.STANDBY;
				}
			}
			else {
				LOG.logError("Hooks not found");
			}	
			break;
			}
		return false;		
	}
	
	/**
	 * Sets the position of the arms
	 * @param solenoidValue is the value to send to both solenoids
	 */
	private void setArms(boolean solenoidValue){
		if (arms.get() != solenoidValue){
			arms.set(solenoidValue);
		}
	}
	
	/**
	 * Sets the position of the ratchet 
	 * @param solenoidValue is the value to send to both solenoids
	 */
	private void setLock(boolean solenoidValue){
		if (ratchet.get() != solenoidValue)
		{
			ratchet.set(solenoidValue);
		}
	}
	
	/**
	 * Sleep time between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}
	
	/**
	 * Writes info about the subsystem to the log 
	 */
	@Override
	protected void writeLog() {
		LOG.logMessage("Current Scaling State" + currentScalingState);
	}
	
	/**
	 *Makes the states for scaling
	 */
	public enum State{
		STANDBY,
		EXTENDING,
		SCALING;

		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "Scaling standby";
			case EXTENDING:
				return "Extending arms";
			case SCALING:
				return "Scaling";
					default:
				return "Unknown scaling state";
			}
		}
	}

	/**
	 * Method that Controls the calls for extending arms  
	 */
	public void extendArms(){
		currentScalingState = State.EXTENDING;
	}
	
	/**
	 * Method that controls the calls for scaling
	 */
	public void scale(){
		currentScalingState = State.SCALING;
		drives.scaleWinch(WINCH_IN_DISTANCE);
	}
	
	/**
	 * Method that estops scaling
	 */
	public void estop(){
		drives.estop();
		currentScalingState = State.STANDBY;
		LOG.logMessage("Scaling ESTOP");
	}
}