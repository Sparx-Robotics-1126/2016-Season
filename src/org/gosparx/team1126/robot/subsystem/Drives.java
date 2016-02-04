package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.sensors.EncoderData;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


/**
 * This class is intended to drive the robot in tank drive
 * @author Meekly
 */
public class Drives extends GenericSubsystem{

	//*********************INSTANCES**********************

	/**
	 * Makes THE drives instance that will be called to use the drives
	 */
	private static Drives drives;

	//*********************MOTOR CONTROLLERS**************

	/**
	 * the controller to the right front motor
	 */
	private CANTalon rightFront;

	/**
	 * the controller to the right back motor
	 */
	private CANTalon rightBack;

	/**
	 * the controller to the left front motor
	 */
	private CANTalon leftFront;

	/**
	 * the controller to the left back motor
	 */
	private CANTalon leftBack;

	//*********************PNEUMATICS****************************

	/**
	 * the solenoid to shift between high and low gear
	 */
	private Solenoid shiftingSol;

	//*********************SENSORS************************

	/**
	 * Used to get the distance the robot has traveled for the left drives 
	 */
	private Encoder encoderLeft;

	/**
	 * Used to get the distance the robot has traveled for the right drives
	 */
	private Encoder encoderRight;

	/**
	 * makes the left encoder data which calculates how far the robot traveled in inches
	 */
	private EncoderData encoderDataLeft;

	/**
	 * makes the right encoder data which calculates how far the robot traveled in inches
	 */
	private EncoderData encoderDataRight;
	
	/**
	 * gyro used to keep ourselves align and to turn in Auto
	 */
	private AnalogGyro gyro;

	//*********************CONSTANTS**********************

	/**
	 * the amount of distance the robot will travel per tick 
	 * equation: Circumference/256(distance per tick)
	 */
	private final double DISTANCE_PER_TICK = 0.0490873852;

	/**
	 * the speed required to shift down in inches per sec, not accurate yet
	 */
	private static final double LOWER_SHIFTING_SPEED = 15;

	/**
	 * the speed required to shift up in inches per sec, not accurate yet
	 */
	//private static final double UPPER_SHIFTING_SPEED = 40;
	private static final double UPPER_SHIFTING_SPEED = 1000000000;

	/**
	 * the time required to pause for shifting in seconds, not accurate yet, in seconds
	 */
	private static final double SHIFTING_TIME = 0.25;

	/**
	 * the speed used during shifting
	 */
	private static final double SHIFTING_SPEED = 0.7;

	/**
	 * solenoid value for low gear
	 */
	private static final boolean LOW_GEAR = true;

	/**
	 * motor speed for stopped
	 */
	private static final double STOP_MOTOR = 0;


	//*********************VARIABLES**********************

	/**
	 * the current average speed between the left and right motors in inches per sec
	 */
	private double currentSpeedAvg;

	/**
	 * current time shifting
	 */
	private double shiftingTime;

	/**
	 * The current state that drives is in
	 */
	private State currentDriveState;

	/**
	 * the wanted distance to travel during autonomous in inches
	 */
	private double wantedAutoDist;

	/**
	 * The speed of which you want to go during autonomous inches per sec
	 */
	private double wantedAutoSpeed;

	/**
	 * The current distance traveled in autonomous in inches
	 */
	private double currentAutoDist;

	/**
	 * The current state that autonomous is in.
	 */
	private State autoState;
	
	/**
	 * traveled left distance in auto
	 */
	private double traveledLeftDistanceAuto;
	
	/**
	 * traveled right distance in auto
	 */
	private double traveledRightDistanceAuto;
	
	/**
	 * power being sent to the left motors
	 */
	private double wantedLeftPower;
	
	/**
	 * power being sent to the right motors
	 */
	private double wantedRightPower;
	
	/**
	 * current speed the left drive is going in auto in inches per sec
	 */
	private double currentLeftSpeed;
	
	/**
	 * current speed the right drive is going in auto in inches per sec
	 */
	private double currentRightSpeed;
	
	/**
	 * set to true if turning left in AUto
	 */
	private boolean leftDirectionAuto;
	
	/**
	 *  value 0 to 360
	 */
	private double turnDegreesAuto;
	
	/**
	 * Variable for the scale functions
	 */
	private State CurrentScaleState;

	/**
	 * Variable for the distance scaled on the left side
	 */
	private double traveledLeftDistanceScale;
	
	/**
	 * Variable for the distance scaled on the right side
	 */
	private double traveledRightDistanceScale;
	
	/**
	 * Variable for the average distance currently scaled
	 */
	private double currentScaleDist;
	
	/**
	 * Variable for the wanted winch in power
	 */
	private double wantedWinchInPower;
	
	/**
	 * Variable for the wanted winch in distance 
	 */
	private double wantedWinchInDistance; 
	
	/**
	 * Creates a drives with normal priority
	 */
	private Drives(){
		super("Drives", Thread.NORM_PRIORITY);
	}

	/**
	 * This creates a drives object with a name and its priority
	 */
	public static synchronized Drives getInstance() {
		if(drives == null){
			drives = new Drives();
		}
		return drives;
	}

	/**
	 * Instantiates all of the objects and gives data to the variables
	 * return: return true it runs once, false continues, should be true
	 */
	@Override
	protected boolean init() {

		//RIGHT
		rightFront = new CANTalon(1);
		//rightBack = new Talon(1);
		encoderRight = new Encoder(1,0);
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK);


		//LEFT
		//leftBack = new Talon(1);
		leftFront = new CANTalon(0);
		encoderLeft = new Encoder(3,2);
		encoderDataLeft = new EncoderData(encoderLeft,DISTANCE_PER_TICK);

		//OTHER
		gyro = new AnalogGyro(1);
		wantedLeftPower = 0;
		wantedRightPower = 0;
		currentDriveState = State.IN_LOW_GEAR;
		shiftingSol = new Solenoid(0);
		wantedAutoDist = 108;
		autoState = State.AUTO_DRIVE;
		turnDegreesAuto = 90;
		leftDirectionAuto = false;
		gyro.calibrate();
		CurrentScaleState = State.SCALING_STANDBY;
		return true;
	}

	/**
	 * Used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		String subsytemName = "Drives";
		LiveWindow.addActuator(subsytemName, "Shifting", shiftingSol);
		LiveWindow.addActuator(subsytemName, "Right Encoder", encoderRight);
		LiveWindow.addActuator(subsytemName, "Right Front Motor", rightFront);
		//LiveWindow.addActuator(subsytemName, "Right Rear Motor", rightBack);
		LiveWindow.addActuator(subsytemName, "Left Front Motor", leftFront);
		//LiveWindow.addActuator(subsytemName, "Left Front Motor", leftBack);
		LiveWindow.addActuator(subsytemName, "Left Encoder", encoderLeft);
	}

	/**
	 * it runs on a loop until returned false, don't return false
	 * is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		encoderDataLeft.calculateSpeed();
		encoderDataRight.calculateSpeed();
		currentLeftSpeed = Math.abs(encoderDataLeft.getSpeed());
		currentRightSpeed = Math.abs(encoderDataLeft.getSpeed());
		currentSpeedAvg = (currentLeftSpeed + currentRightSpeed)/2;

		switch(currentDriveState){

		case IN_LOW_GEAR:
			if(Math.abs(currentSpeedAvg)>= UPPER_SHIFTING_SPEED){
				System.out.println("SHIFTING HIGH!");
				shiftingTime = Timer.getFPGATimestamp();
				currentDriveState = State.SHIFTING_HIGH;
				if(currentSpeedAvg < 0){
					wantedLeftPower = (SHIFTING_SPEED * -1);
					wantedRightPower = (SHIFTING_SPEED * -1);
				}else{
					wantedLeftPower = (SHIFTING_SPEED);
					wantedRightPower = (SHIFTING_SPEED);
				}
			}
			break;

		case SHIFTING_HIGH:
			shiftingSol.set(!LOW_GEAR);
			if(Timer.getFPGATimestamp() >= shiftingTime + SHIFTING_TIME){
				currentDriveState = State.IN_HIGH_GEAR;
			}

			break;

		case IN_HIGH_GEAR:
			if(Math.abs(currentSpeedAvg) <= LOWER_SHIFTING_SPEED){
				System.out.println("SHIFTING LOW!");
				shiftingTime = Timer.getFPGATimestamp();
				currentDriveState = State.SHIFTING_LOW;
				if(currentSpeedAvg < 0){
					wantedLeftPower = (SHIFTING_SPEED * -1);
					wantedRightPower = (SHIFTING_SPEED * -1);
				}else{
					wantedLeftPower = (SHIFTING_SPEED);
					wantedRightPower = (SHIFTING_SPEED);
				}
			}

			break;

		case SHIFTING_LOW:
			shiftingSol.set(LOW_GEAR);
			if(Timer.getFPGATimestamp() >= shiftingTime + SHIFTING_SPEED){
				currentDriveState = State.IN_LOW_GEAR;
			}
			break;
		default: System.out.println("Error, current drives state is: " + currentDriveState);
		}
	
		switch(autoState){
		case AUTO_STANDBY:
			break;
		case AUTO_DRIVE:
			traveledLeftDistanceAuto = Math.abs(encoderDataLeft.getDistance());
			traveledRightDistanceAuto = Math.abs(encoderDataRight.getDistance());
			currentAutoDist = (traveledLeftDistanceAuto + traveledRightDistanceAuto)/2;
			// needs explanation
			wantedAutoSpeed = (.8/10)*(Math.sqrt(Math.abs(wantedAutoDist - currentAutoDist)));
			wantedAutoSpeed = wantedAutoSpeed > 1 ? 1: wantedAutoSpeed;
			// need explanation
			wantedAutoSpeed = wantedAutoSpeed < Math.PI/18 ? Math.PI/18: wantedAutoSpeed;
			wantedAutoSpeed = -wantedAutoSpeed;
			
			// Create a constant for the 0.4 and explain (include units)
			if(Math.abs(currentLeftSpeed-currentRightSpeed) < .4){
				wantedLeftPower = wantedAutoSpeed;
				wantedRightPower = wantedAutoSpeed;
			}else if(currentLeftSpeed < currentRightSpeed){
				// Why 6/5 explain and make into a constant (include units)
				wantedLeftPower = wantedAutoSpeed * (6/5) < 1 ? wantedAutoSpeed *(6/5): 1;
				wantedRightPower = wantedAutoSpeed;
			}else {
				// Why 6/5 explain and make into a constant (include units)
				wantedRightPower = wantedAutoSpeed * (6/5) < 1 ? wantedAutoSpeed *(6/5): 1;
				wantedLeftPower = wantedAutoSpeed;
			}
			
			if(Math.abs(currentAutoDist) >= Math.abs(wantedAutoDist)){
				wantedLeftPower = STOP_MOTOR;
				wantedRightPower = STOP_MOTOR;
				encoderDataLeft.reset();
				encoderDataRight.reset();
				autoState = State.AUTO_STANDBY;
				System.out.println("WE'RE DONE I HOPE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}
			break;
			
		case AUTO_TURN:
			if(gyro.getAngle() != turnDegreesAuto){
				if(leftDirectionAuto){
					wantedLeftPower = -.1;
					wantedRightPower = .1;
				}else {
					wantedLeftPower = .1;
					wantedRightPower = -.1;
				}
					
			}else
				autoState = State.AUTO_STANDBY;
			break;
		default: System.out.println("Error, auto state is: " + autoState);
		}
		
		switch (CurrentScaleState)
		{
		case SCALE_SCALING: {
			encoderRight.reset();
			encoderLeft.reset();
			traveledLeftDistanceScale = Math.abs(encoderDataLeft.getDistance());
			traveledRightDistanceScale = Math.abs(encoderDataRight.getDistance());
			currentScaleDist = (traveledLeftDistanceScale + traveledRightDistanceScale)/2;
			wantedRightPower = wantedWinchInPower;
			wantedLeftPower = wantedWinchInPower;
			if(Math.abs(currentScaleDist) >= Math.abs(wantedWinchInDistance)){
				wantedLeftPower = STOP_MOTOR;
				wantedRightPower = STOP_MOTOR;
				CurrentScaleState = State.SCALING_STANDBY;
			}
			break; 
			}
		case SCALING_STANDBY:
			break;
		default:
			break;
		}

		leftFront.set(-wantedLeftPower);
		//leftBack.set(leftPower);
		rightFront.set(wantedRightPower);
		//rightBack.set(rightPower);

		return false;
	}

	/**
	 * how long the class "rests" until it is called again
	 * return: how long it rests in milliseconds
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * writes a log to the console every 5 seconds
	 */
	@Override
	protected void writeLog() {
				LOG.logMessage("The wanted powers are (left, right): " + wantedLeftPower + ", " + wantedRightPower);
				LOG.logMessage("The speeds are (left, right): " + Math.abs(encoderDataLeft.getSpeed()) +", " + Math.abs(encoderDataRight.getSpeed()));
				LOG.logMessage("We are currently in this state-------- " + currentDriveState);
				LOG.logMessage("We have gone this far!! " + (Math.abs(encoderDataLeft.getDistance()) + Math.abs(encoderDataRight.getDistance()))/2);
				LOG.logMessage("The current auto distance left is " + (Math.abs(wantedAutoDist) - Math.abs(currentAutoDist)));
				LOG.logMessage("The current winch in distance left is " + (Math.abs(wantedWinchInDistance) - Math.abs(currentScaleDist)));
				LOG.logMessage("We are currently in this Sclaeing state-------- " + CurrentScaleState);
	}

	/**
	 * is used to get the power from the joysticks 
	 * @param left the left joystick input from -1 to 1
	 * @param right the right joystick input from -1 to 1
	 */
	public void setPower(double left, double right) {
		wantedLeftPower = left;
		wantedRightPower = right;
	}
	/**
	 *Makes the states for drives
	 */
	public enum State{
		IN_LOW_GEAR,
		SHIFTING_LOW,
		IN_HIGH_GEAR,
		SHIFTING_HIGH,
		AUTO_DRIVE,
		AUTO_TURN,
		AUTO_STANDBY, 
		SCALING_STANDBY, 
		SCALE_SCALING;

		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case IN_LOW_GEAR:
				return "In low gear";
			case SHIFTING_LOW:
				return "Shifting Low";
			case IN_HIGH_GEAR:
				return "In high gear";
			case SHIFTING_HIGH:
				return "Shifting high";
			case AUTO_DRIVE:
				return "In Auto Drive";
			case AUTO_STANDBY:
				return "In Auto Standby";
			default:
				return "Error :(";
			}
		}
	}

	/**
	 * drives the robot to a certain distance
	 * @param length: the length you want it to go
	 * @param speed: the speed you want it to go
	 */
	public void driveWantedDistance(double length){
		wantedAutoDist = length;
		autoState = State.AUTO_DRIVE;
	}
	
	/**
	 * 
	 * @param left true turn left, false turn right
	 * @param angle the angle you want to be at from 0-360
	 */
	public void turn(boolean left, double angle){
		leftDirectionAuto = left;
		turnDegreesAuto = angle;
	}
	
	/**
	 * Called by Scaling methods to set desired scaling state
	 * @param wantedScaleState
	 */
	public void setScalingFunction(State wantedScaleState){
		CurrentScaleState = wantedScaleState;
	}

	/**
	 * Called by Scaling when arms have been extended and we want to winch out 
	 * @param distanceToScale= the distance we need to scale
	 * @param winchInPower= the power to winch in
	 */
	public void scaleWinch(double distanceToScale, double winchInPower) {
		setScalingFunction(State.SCALE_SCALING);
		wantedWinchInDistance = distanceToScale;
		wantedWinchInPower = winchInPower;
	}

	/**
	 * Checks to see if scaling is done 
	 * @return
	 */
	public boolean isScaleScalingDone() {
		return (CurrentScaleState == State.SCALING_STANDBY);
	}
}