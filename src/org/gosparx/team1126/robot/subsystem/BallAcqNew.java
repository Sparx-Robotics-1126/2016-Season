package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;
import org.gosparx.team1126.robot.sensors.MagnetSensor;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallAcqNew extends GenericSubsystem{

	//****************************Constants******************

	/**
	 * deadband for arm angles.
	 */
	private final double DEADBAND = .5;

	/**
	 * Distance per tick
	 */
	private final double DISTANCE_PER_TICK = (0.1690141 * 4);

	/**
	 * The higher arm power
	 */
	private final double HIGH_ARM_POWER = .5;

	/**
	 * The amount of time we want the flipper to stay up after firing (in seconds)
	 */
	private static final double WAIT_FIRE_TIME = 0.25;

	/**
	 * The power to use when kicking the ball out of the robot
	 */
	private static final double HIGH_ROLLER_POWER = 1;

	/**
	 * The power to use when dropping the ball to a teammate blah
	 * The power to use when holding the arms at acquire position.
	 */
	private static final double HOLDING_POWER = 0.05;

	/**
	 * for the flipper and circ pivot long/a
	 */
	private static final boolean CONTRACTED_LONG = false;

	/**
	 * same as above
	 */
	private static final boolean EXTENDED_LONG = !CONTRACTED_LONG;

	/**
	 * for the circ pivot short/b
	 */
	private static final boolean CONTRACTED_SHORT = true;

	/**
	 * same as above
	 */
	private static final boolean EXTENDED_SHORT = !CONTRACTED_SHORT;
	
	/**
	 * the maximum angle the arms can go
	 */
	private static final int MAX_ANGLE = 125;

	/**
	 * the power for the left roller
	 */
	private static final int LEFT_ROLLER_PDP = 10;

	/**
	 * the power for the right roller
	 */
	private static final int RIGHT_ROLLER_PDP = 11;

	//*****************************Objects*******************

	/**
	 * the instance of BallAcqNew
	 */
	private static BallAcqNew acqui;

	/**
	 * the current state of the arms
	 */
	private ArmState currentArmState;

	/**
	 * the current state of the flipper
	 */
	private FlipperState currentFlipperState;

	/**
	 * the current state of the roller
	 */
	private RollerState currentRollerState;

	/**
	 * the rightmost arm motor
	 */
	private CANTalon armMotorRight;

	/**
	 * the leftmost arm motor
	 */
	private CANTalon armMotorLeft;

	/**
	 * the rightmost roller motor
	 */
	private CANTalon rollerMotorRight;

	/**
	 * the leftmost roller motor
	 */
	private CANTalon rollerMotorLeft;

	/**
	 * the solenoid of the flipper
	 */
	private Solenoid flipper;

	/**
	 * the solenoid for the long pneumatic cylinders on the arms
	 * (no longer needed as of now)
	 */
	private Solenoid circPivotLong;

	/**
	 * the solenoid for the short pneumatic cylinders on the arms
	 * (no longer needed as of now)
	 */
	private Solenoid circPivotShort;

	/**
	 * the rightmost arm encoder
	 */
	private Encoder armEncoderRight;

	/**
	 * the leftmost arm encoder
	 */
	private Encoder armEncoderLeft;

	/**
	 * the encoder data for the rightmost arm encoder
	 */
	private EncoderData armEncoderDataR;

	/**
	 * the encoder data for the leftmost arm encoder
	 */
	private EncoderData armEncoderDataL;

	/**
	 * Magnetic sensor for the arm's home position
	 */
	private MagnetSensor armHomeSwitch;

	/**
	 * the photo electric sensor to see if the ball is in
	 */
	private DigitalInput ballEntered;

	/**
	 * the photo electric sensor to see if the ball is fully in the robot.
	 */
	private DigitalInput ballFullyIn;

	/**
	 * the power distribution panel
	 */
	private PowerDistributionPanel pdp;

	//************************Variables*********************

	/**
	 * the wanted angle of the arm
	 */
	private double wantedArmAngle;

	/**
	 * the time we fired the ball during the match (in seconds)
	 */
	private double timeFired;

	/**
	 * The wanted power of the right roller motor.
	 */
	private double wantedPowerRR;

	/**
	 * The wanted power of the left roller motor
	 */
	private double wantedPowerRL;

	/**
	 * The wanted power of the right arm motor
	 */
	private double wantedArmPowerRight;

	/**
	 * The wanted power of the left arm motor
	 */
	private double wantedArmPowerLeft;

	/**
	 * the average distance traveled between the two arm encoders
	 */
	private double averageArmDistance;

	/**
	 * the distance traveled by the left encoder
	 */
	private double leftDistance;

	/**
	 * the distance traveled by the right encoder
	 */
	private double rightDistance;

	/**
	 * Do we want to reverse the roller direction?
	 */
	private boolean reverseRollers;

	/**
	 * Whether the arms are in their home position
	 */
	private boolean armHome;

	/**
	 * Whether we are firing or not
	 */
	private boolean firing;

	/**
	 * temporary
	 */
	private double highAmp = 0;

	private BallAcqNew() {
		super("BallAcqNew", Thread.NORM_PRIORITY);
	}

	/**
	 * makes sure that there is only one instance of BallAcq
	 * @return a BallAcq object
	 */
	public static synchronized BallAcqNew getInstance(){
		if(acqui == null){
			acqui = new BallAcqNew();
		}
		return acqui;
	}
	
	/**
	 * Instantiates all of the objects and gives data to the variables
	 * @return true if it runs once and false continues; should be true
	 */
	@Override
	protected boolean init() {
		currentArmState = ArmState.ROTATE_FINDING_HOME;
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		armMotorRight = new CANTalon(IO.CAN_ACQ_SHOULDER_R);
		armMotorLeft = new CANTalon(IO.CAN_ACQ_SHOULDER_L);
		rollerMotorRight = new CANTalon(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorLeft = new CANTalon(IO.CAN_ACQ_ROLLERS_L);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivotLong = new Solenoid(IO.PNU_CIRCLE_POSITION_A);
		circPivotShort = new Solenoid(IO.PNU_CIRCLE_POSITION_B);
		armEncoderRight = new Encoder(IO.DIO_SHOULDER_ENC_RIGHT_A, IO.DIO_SHOULDER_ENC_RIGHT_B);
		armEncoderLeft = new Encoder(IO.DIO_SHOULDER_ENC_LEFT_A, IO.DIO_SHOULDER_ENC_LEFT_B);
		armEncoderDataR = new EncoderData(armEncoderRight, DISTANCE_PER_TICK);
		armEncoderDataL = new EncoderData(armEncoderLeft, DISTANCE_PER_TICK);
		armHomeSwitch = new MagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME, true);
		ballEntered = new DigitalInput(IO.DIO_PHOTO_BALL_ENTER);
		ballFullyIn = new DigitalInput(IO.DIO_LIMIT_BALL_IN);
		pdp = new PowerDistributionPanel();
		wantedArmAngle = 0;
		timeFired = 0;
		wantedPowerRR = 0;
		wantedPowerRL = 0;
		wantedArmPowerRight = 0;
		wantedArmPowerLeft = 0;
		averageArmDistance = 0;
		leftDistance = 0;
		rightDistance = 0;
		armHome = false;
		firing = false;
		reverseRollers = false;
		return false;
	}

	/**
	 * used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "BallAcq1";
		String subsyst = "BallAcq2";
		String sub = "BallAcq3";
		LiveWindow.addActuator(subsystemName, "Right Arm Motor", armMotorRight);
		LiveWindow.addActuator(subsystemName, "Left Arm Motor", armMotorLeft);
		LiveWindow.addActuator(subsystemName, "Right Roller Motor", rollerMotorRight);
		LiveWindow.addActuator(subsystemName, "Left Roller Motor", rollerMotorLeft);
		LiveWindow.addActuator(sub, "Right Arm Encoder", armEncoderRight);
		LiveWindow.addActuator(sub, "Left Arm Encoder", armEncoderLeft);
		LiveWindow.addActuator(subsyst, "Flipper", flipper);
		LiveWindow.addActuator(subsyst, "Circular Pivot Long", circPivotLong);
		LiveWindow.addActuator(subsyst, "Circular Pivot Short", circPivotShort);
		LiveWindow.addSensor(subsyst, "Ball Entered Sensor", ballEntered);
		LiveWindow.addSensor(subsyst, "Ball Fully In Sensor", ballFullyIn);

	}

	/**
	 * it runs on a loop until returned false, don't return true
	 * it is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		leftDistance = armEncoderLeft.getDistance();
		rightDistance = -armEncoderRight.getDistance();
		armHome = armHomeSwitch.isTripped();
		switch(currentArmState){
		case STANDBY:
			wantedArmPowerRight = 0;
			wantedArmPowerLeft = 0;
			break;
		case ROTATE:
			if(!((leftDistance > wantedArmAngle - DEADBAND) && 
					(leftDistance < wantedArmAngle + DEADBAND))){
				if(wantedArmAngle > leftDistance){
					wantedArmPowerLeft = -HIGH_ARM_POWER;
				}else{
					wantedArmPowerLeft = HIGH_ARM_POWER;
				}
			}else{
				wantedArmPowerLeft = 0;
			}
			if(!((rightDistance > wantedArmAngle - DEADBAND) &&
					(rightDistance < wantedArmAngle + DEADBAND))){
				if(wantedArmAngle > rightDistance){
					wantedArmPowerRight = -HIGH_ARM_POWER;
				}else{
					wantedArmPowerRight = HIGH_ARM_POWER;
				}
			}else{
				wantedArmPowerRight = 0;
			}
			if(wantedArmPowerRight == 0 && wantedArmPowerLeft == 0){
				currentArmState = ArmState.STANDBY;
			}
			break;
		case ROTATE_FINDING_HOME:
			if(armHome){
				currentArmState = ArmState.STANDBY;
				currentRollerState = RollerState.STANDBY;
				armMotorLeft.set(0);
				armMotorRight.set(0);
				wantedArmPowerRight = 0;
				wantedArmPowerLeft = 0;
				armEncoderRight.reset();
				armEncoderLeft.reset();
			}else{
				wantedArmAngle = 0;
				wantedArmPowerRight = HIGH_ARM_POWER;
				wantedArmPowerLeft = HIGH_ARM_POWER;
			}
			break;
		case HOLDING:
			if(armEncoderDataL.getDistance() < wantedArmAngle){
				wantedArmPowerRight = 0;
				wantedArmPowerLeft = 0;
			}else{
				wantedArmPowerLeft = HOLDING_POWER;
				wantedArmPowerRight = HOLDING_POWER;
			}
			break;
		case OP_CONTROL:
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}

		switch(currentFlipperState){
		case STANDBY:
			flipper.set(CONTRACTED_LONG);
			break;
		case FIRING:
			if(!firing){
				flipper.set(EXTENDED_LONG);
				timeFired = Timer.getFPGATimestamp();
				firing = true;
			}else if(Timer.getFPGATimestamp() >= timeFired + WAIT_FIRE_TIME && firing){
				flipper.set(CONTRACTED_LONG);
				firing = false;
				LOG.logMessage("Succeeded in firing the flipper");
				currentFlipperState = FlipperState.STANDBY;
			}
			break;
		case HOLD_UP:
			flipper.set(EXTENDED_LONG);
			break;	
		default:
			System.out.println("INVALID STATE: " + currentFlipperState);
			break;
		}
		switch(currentRollerState){
		case STANDBY:
			wantedPowerRR = 0;
			wantedPowerRL = 0;
			break;
		case ROLLER_ON:
			if(pdp.getCurrent(LEFT_ROLLER_PDP) > highAmp){
				highAmp = pdp.getCurrent(LEFT_ROLLER_PDP);
				System.out.println("new high: " + highAmp);
			}else if(pdp.getCurrent(RIGHT_ROLLER_PDP) > highAmp){
				highAmp = pdp.getCurrent(RIGHT_ROLLER_PDP);
				System.out.println("new high: " + highAmp);
			}

			wantedPowerRR = HIGH_ROLLER_POWER;
			wantedPowerRL = HIGH_ROLLER_POWER;
			break;
		default:
			System.out.println("INVALID STATE: " + currentRollerState);
			break;
		}
		/*
		if((armHome && wantedArmPowerLeft < 0) ||
				(leftDistance > MAX_ANGLE && wantedArmPowerLeft > 0 && rightDistance > MAX_ANGLE)){
			LOG.logMessage("You are trying to break the arm");
			currentArmState = ArmState.STANDBY;
			wantedArmPowerRight = 0;
			wantedArmPowerLeft = 0;
		}
		 */
		wantedPowerRR = (reverseRollers) ? wantedPowerRR * -1: wantedPowerRR;
		wantedPowerRL = (reverseRollers) ? wantedPowerRL * -1: wantedPowerRL;
		rollerMotorRight.set(wantedPowerRR);
		rollerMotorLeft.set(-wantedPowerRL);
		armMotorRight.set(-wantedArmPowerRight);
		armMotorLeft.set(wantedArmPowerLeft);
		SmartDashboard.putBoolean("Ball Entered?", ballEntered.get());
		SmartDashboard.putBoolean("Ball in Flipper?", ballFullyIn.get());
		return false;
	}

	/**
	 * sets the power based off the controller
	 * @param pow the controller input
	 */
	public void setArmPower(double pow){
		if(currentArmState == ArmState.OP_CONTROL){
			wantedArmPowerLeft = pow;
			wantedArmPowerRight = pow;
		}
	}

	/**
	 * 
	 * @param opControl
	 * sets the arm state for operator control
	 */
	public void setOpControl(boolean opControl){
		currentArmState = (opControl) ? ArmState.OP_CONTROL : ArmState.STANDBY;
	}

	/**
	 * sets the home position
	 */
	public void setHome(){
		currentArmState = ArmState.ROTATE_FINDING_HOME;
		currentRollerState = RollerState.STANDBY;
		reverseRoller(false);
	}

	/**
	 * home with rollers on
	 */
	public void homeRollers(){
		currentArmState = ArmState.ROTATE_FINDING_HOME;
		currentRollerState = RollerState.ROLLER_ON;
		reverseRoller(false);
	}

	/**
	 * acquires the ball from the ground to the flipper
	 */
	public void acquireBall(){
		wantedArmAngle = 100;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.ROLLER_ON;
		reverseRoller(false);
	}

	/**
	 * raise the gate
	 */
	public void raiseGate(){
		wantedArmAngle = 0;
		currentArmState = ArmState.ROTATE;
		reverseRoller(false);
		currentRollerState = RollerState.STANDBY;
	}

	/**
	 * goes to the angle we need to be at when crossing the sally port 
	 */
	public void goToSallyPortPosition(){
		wantedArmAngle = 85;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.STANDBY;
		//circPivotLong.set(CONTRACTED_LONG);
		//circPivotShort.set(EXTENDED_SHORT);
		flipper.set(EXTENDED_LONG);
		reverseRoller(false);
	}
	/**
	 * moves to low bar position
	 */
	public void goToLowBarPosition(){
		wantedArmAngle = 125;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.STANDBY;
		//		circPivotLong.set(CONTRACTED_LONG);
		//		circPivotShort.set(EXTENDED_SHORT);
		flipper.set(EXTENDED_LONG);
		reverseRoller(false);
	}

	/**
	 * moves the arms to an okay position so the arms aren't in to the way of the scaling arms
	 * @return true if the arms are out of the way, false if they are in the way
	 */
	public boolean moveToScale(){
		wantedArmAngle = 125;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.STANDBY;
		reverseRoller(false);
		if(averageArmDistance > wantedArmAngle - DEADBAND && 
				averageArmDistance < wantedArmAngle + DEADBAND){
			return true;
		}else
			return false;
	}


	/**
	 * fires the flipper
	 * @return true if the flipper fires and false if the flipper is already firing
	 */
	public boolean fire(){
		if(currentFlipperState == FlipperState.FIRING)
			return false;
		else{
			currentFlipperState = FlipperState.FIRING;
			return true;
		}
	}

	/**
	 * Toggles roller.
	 * @return true if the roller is on, false if the roller is off
	 */
	public boolean toggleRoller(){
		if(currentRollerState == RollerState.ROLLER_ON){
			currentRollerState = RollerState.STANDBY;
			return false;
		}else{
			currentRollerState = RollerState.ROLLER_ON;
			return true;
		}
	}

	/**
	 * Reverses roller.
	 */
	public void reverseRoller(){
		reverseRollers = !reverseRollers;
	}

	/**
	 * Set the reverse of the rollers
	 */
	public void reverseRoller(boolean rev){
		reverseRollers = rev;
	}

	/**
	 * stops the entire system
	 */
	public void stopAll(){
		currentArmState = ArmState.STANDBY;
		flipper.set(CONTRACTED_LONG);
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		reverseRoller(false);
	}

	/**
	 * time to rest the system between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * for log messages
	 */
	@Override
	protected void writeLog() {
		LOG.logMessage("Current arm state: " + currentArmState);
		LOG.logMessage("Current roller state: " + currentRollerState);
		LOG.logMessage("Current flipper state: " + currentFlipperState);
		LOG.logMessage("Right Roller Motor speed:" + rollerMotorRight.get());
		LOG.logMessage("Left Roller Motor speed:" + rollerMotorLeft.get());
		LOG.logMessage("Arm Motor Right speed:" + armMotorRight.get());
		LOG.logMessage("Arm Motor Left speed:" + armMotorLeft.get());
		LOG.logMessage("Arm Home Sensor:" + armHomeSwitch.isTripped());
		LOG.logMessage("Ball Entered Sensor:" + ballEntered.get());
		LOG.logMessage("Ball Fully In Sensor:" + ballFullyIn.get());
		LOG.logMessage("The Arm Left Degrees: " + armEncoderDataL.getDistance());
		LOG.logMessage("The Arm Right Degrees: " + -armEncoderDataR.getDistance());
	}
	
	public enum ArmState{
		STANDBY,
		ROTATE,
		ROTATE_FINDING_HOME,
		HOLDING,
		ACQUIRING,
		OP_CONTROL;

		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "The arms are in standby";
			case ROTATE:
				return "The arms are rotating";
			case ROTATE_FINDING_HOME:
				return "The arms are going home";
			case HOLDING:
				return "The arms are being held";
			case ACQUIRING:
				return "We are acquiring";
			case OP_CONTROL:
				return "The operator is in control";
			default:
				return "Error :( The arms are in " + this;
			}
		}
	}
	public enum FlipperState{
		STANDBY,
		FIRING,
		HOLD_UP;

		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){	
			case STANDBY:
				return "The flipper is in Standby";
			case FIRING:
				return "The flipper is firing";
			case HOLD_UP:
				return "The flipper is being held up";
			default:
				return "Error :( The flipper is in " + this;
			}
		}
	}
	public enum RollerState{
		STANDBY,
		ROLLER_ON;
		
		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "The roller is in Standby";
			case ROLLER_ON:
				return "The roller is on";
			default:
				return "Error :( The roller is in " + this;
			}
		}
	}
}