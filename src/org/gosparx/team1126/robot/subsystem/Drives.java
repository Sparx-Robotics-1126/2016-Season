package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is intended to drive the robot in tank drive
 * @author Michael
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

	/**
	 * the solenoid used to engage and disengage the pto
	 */
	//private Solenoid ptoSol;

	//*********************SENSORS************************

	/**
	 * 
	 */
	private DigitalInput leftA;

	/**
	 * 
	 */
	private DigitalInput leftB;

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
	private AnalogGyro angleGyro;

	//*********************CONSTANTS**********************

	/**
	 * the amount of distance the robot will travel per tick 
	 * equation: Circumference/256(distance per tick)
	 */
	//private final double DISTANCE_PER_TICK = ((1/10)*Math.PI*6)/256;
	//private final double DISTANCE_PER_TICK = 0.007363108;
	private final double DISTANCE_PER_TICK = 0.00689;

	/**
	 * the speed required to shift down in inches per sec, not accurate yet
	 */
	private static final double LOWER_SHIFTING_SPEED = 20;

	/**
	 * the speed required to shift up in inches per sec, not accurate yet
	 */
	private static final double UPPER_SHIFTING_SPEED = 45;

	/**
	 * the time required to pause for shifting in seconds, not accurate yet, in seconds
	 */
	private static final double SHIFTING_TIME = 0.125;

	/**
	 * the power used during shifting
	 */
	private static final double SHIFTING_POWER = 0.4;

	/**
	 * solenoid value for low gear
	 */
	private static final boolean LOW_GEAR = false;

	/**
	 * motor speed for stopped
	 */
	private static final double STOP_MOTOR = 0;

	/**
	 * The distance in inches where drives straight has been achieved +-
	 */
	private static final double MAX_TURN_ERROR = .1;

	/**s
	 * the speed the left & right drives can be off while scaling in inches
	 */
	private static final double MAX_SCALE_SPEED_OFF = 0.2;

	/**
	 * the ramping to increase speed if one side is off
	 */
	private static final double FIX_SPEED_SCALE_RAMPING = 51.0/50.0;

	/**
	 * the ramping to increase speed if one side is off
	 */
	private static final double FIX_SPEED_DRIVE_RAMPING = 24.0/20.0;

	/**
	 * the max distance in inches that drives can be off while in autoDrive
	 */
	private static final double MAX_OFF_DISTANCE_AUTO = 0.5;

	/**
	 * The minimum speed drives will go during auto
	 */
	private static final double MIN_AUTO_DRIVE_SPEED = .4;

	/**
	 * The minimum speed drives will go while scaling
	 */
	private static final double MIN_SCALE_SPEED = Math.PI/8.0;

	/**
	 * The number of which to ramp by for auto drive
	 */
	private static final double AUTO_DRIVE_RAMPING = 31.0/500.0;

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
	private DriveState currentDriveState;

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
	private AutoState autoState;

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
	 *  Negative for left turn, positive right turn
	 */
	private double turnDegreesAuto;

	/**
	 * if true the scaling is done
	 */
	private boolean scalingDone = false;

	/**
	 * solenoid value for engage pto
	 */
	private boolean engagePto = false;

	/**
	 * if true, then the operator is in control of the scaling power
	 */
	private boolean scaleOpControl = false;

	/**
	 * if true, it will shift either up or down
	 */
	private boolean toggleShift = false;

	/**
	 * if true, the driver can manually shift
	 */
	private boolean driverShift = false;

	/**
	 * Gets the instance of scaling
	 */
	private Scaling scaling;

	/**
	 * true if we want to start scaling
	 */
	private boolean wantToScale = false;

	/**
	 * the power sent by the joystick
	 */
	private double controlsLeftPower;

	/**
	 * the power sent by the joystick
	 */
	private double controlsRightPower;

	/**
	 * Hold in first
	 */
	private boolean holdFirst;

	private boolean scale;

	//***************************************ALEX'S AUTO DEF*****************************************

	/**
	 * Current part of the defense crossing that we are doing.
	 */
	private AutoState defState;

	/**
	 * What angle are we calling "flat" on the ground.
	 */
	private final double FLAT_TOL = 0;

	/**
	 * What is the angle of the ramp.
	 */
	private final double RAMP_ANGLE = -5;

	/**
	 * What speed do we go to reach the defense
	 */
	private final double REACH_SPEED = -.5;

	/**
	 * The speed we go when crossing the defense
	 */
	private final double CROSS_SPEED = -.75;

	/**
	 * The speed we go when we are coming down
	 */
	private final double COME_DOWN_SPEED = -.5;

	private final double SHIFT_MIN_BETWEEN = .5;

	/**
	 * The gyro that measures tilt.
	 */
	private AnalogGyro tiltGyro;

	/*****************************************END AUTO DEF*************************************/
	/**
	 * Variable for the scale functions
	 */
	private ScalingState currentScaleState;

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

	private double shiftStartTime;
	
	private double scalingStartTime;

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
		rightFront = new CANTalon(IO.CAN_DRIVES_RIGHT_FRONT);
		rightBack = new CANTalon(IO.CAN_DRIVES_RIGHT_BACK);
		//TODO:: real robot has it A, B.
		encoderRight = new Encoder(IO.DIO_RIGHT_DRIVES_ENC_A,IO.DIO_RIGHT_DRIVES_ENC_B);
		//encoderRight = new Encoder(IO.DIO_RIGHT_DRIVES_ENC_B,IO.DIO_RIGHT_DRIVES_ENC_A);
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK);
		//LEFT
		leftBack = new CANTalon(IO.CAN_DRIVES_LEFT_BACK);
		leftBack.setInverted(true);
		leftFront = new CANTalon(IO.CAN_DRIVES_LEFT_FRONT);
		leftFront.setInverted(true);
		//TODO:: same as right encoder
		//encoderLeft = new Encoder(IO.DIO_LEFT_DRIVES_ENC_A,IO.DIO_LEFT_DRIVES_ENC_B);
		leftA = new DigitalInput(IO.DIO_LEFT_DRIVES_ENC_A);
		leftB = new DigitalInput(IO.DIO_LEFT_DRIVES_ENC_B);
		encoderLeft = new Encoder(leftA, leftB);
		//encoderLeft = new Encoder(IO.DIO_LEFT_DRIVES_ENC_B,IO.DIO_LEFT_DRIVES_ENC_A);
		encoderDataLeft = new EncoderData(encoderLeft,-DISTANCE_PER_TICK);
		//OTHER
		angleGyro = new AnalogGyro(IO.ANALOG_IN_ANGLE_GYRO);
		angleGyro.calibrate();
		wantedLeftPower = 0;
		wantedRightPower = 0;
		currentDriveState = DriveState.IN_LOW_GEAR;
		shiftingSol = new Solenoid(IO.PNU_SHIFTER);
		//		ptoSol = new Solenoid(IO.PNU_PTO);
		autoState = AutoState.AUTO_STANDBY;
		currentScaleState = ScalingState.SCALING_STANDBY;
		defState = AutoState.AUTO_DEF;
		tiltGyro = new AnalogGyro(IO.ANALOG_IN_TILT_GYRO);
		tiltGyro.calibrate();
		scaling = Scaling.getInstance(); 
		scale = false;
		holdFirst = false;

		return true;
	} 

	/**
	 * Used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemMotorName = "DrivesMotors";
		String subsystemSensorName = "DrivesSensors";
		LiveWindow.addSensor(subsystemSensorName, "RightEncoder", encoderRight);
		LiveWindow.addSensor(subsystemSensorName, "LeftEncoder", encoderLeft);
		LiveWindow.addSensor(subsystemSensorName, "angleGyro", angleGyro);
		LiveWindow.addSensor(subsystemSensorName, "tiltGyro", tiltGyro);
		LiveWindow.addActuator(subsystemMotorName, "Shifting", shiftingSol);
		//	LiveWindow.addActuator(subsystemMotorName, "ptoSol", ptoSol);
		LiveWindow.addActuator(subsystemMotorName, "RightFrontMotor", rightFront);
		LiveWindow.addActuator(subsystemMotorName, "RightBackMotor", rightBack);
		LiveWindow.addActuator(subsystemMotorName, "LeftFrontMotor", leftFront);
		LiveWindow.addActuator(subsystemMotorName, "LeftBackMotor", leftBack);
	}

	/**
	 * it runs on a loop until returned false, don't return true
	 * is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		//TODO: look for negations for encoder and drives motors
		wantedLeftPower = controlsLeftPower;
		wantedRightPower = controlsRightPower;
		encoderDataLeft.calculateSpeed();
		encoderDataRight.calculateSpeed();
		currentLeftSpeed = encoderDataLeft.getSpeed();
		currentRightSpeed = encoderDataRight.getSpeed();
		currentSpeedAvg = (currentLeftSpeed + currentRightSpeed)/2;

		switch(currentDriveState){
		case IN_LOW_GEAR:
			if(!holdFirst){
				if(driverShift){
					System.out.println(toggleShift + "in low gear");
					if((toggleShift)){
						System.out.println("SHIFTING HIGH TOGGLE!");
						toggleShift = false;
						shiftingTime = Timer.getFPGATimestamp();
						currentDriveState = DriveState.SHIFTING_HIGH;
						if(currentSpeedAvg > 0 && Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
							wantedLeftPower = (SHIFTING_POWER * -1);
							wantedRightPower = (SHIFTING_POWER * -1);
						}else if(Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
							wantedLeftPower = (SHIFTING_POWER);
							wantedRightPower = (SHIFTING_POWER);
						}
					}
				}else{
					if((Math.abs(currentSpeedAvg)>= UPPER_SHIFTING_SPEED && shiftStartTime + SHIFT_MIN_BETWEEN < Timer.getFPGATimestamp()) && !scale){
						System.out.println("SHIFTING HIGH!");
						shiftingTime = Timer.getFPGATimestamp();
						shiftStartTime = Timer.getFPGATimestamp();
						currentDriveState = DriveState.SHIFTING_HIGH;
						if(currentSpeedAvg > 0 && Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
							wantedLeftPower = (SHIFTING_POWER * -1);
							wantedRightPower = (SHIFTING_POWER * -1);
						}else if(Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
							wantedLeftPower = (SHIFTING_POWER);
							wantedRightPower = (SHIFTING_POWER);
						}
					}
				}
			}
			break;

		case SHIFTING_HIGH:
			shiftingSol.set(!LOW_GEAR);
			if((Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER)){
				if(currentSpeedAvg > 0){
					wantedLeftPower = (SHIFTING_POWER * -1);
					wantedRightPower = (SHIFTING_POWER * -1);
				}else{
					wantedLeftPower = (SHIFTING_POWER);
					wantedRightPower = (SHIFTING_POWER);
				}
			}
			if(Timer.getFPGATimestamp() >= shiftingTime + SHIFTING_TIME){
				currentDriveState = DriveState.IN_HIGH_GEAR;
			}

			break;
		case IN_HIGH_GEAR:
			if(driverShift){
				System.out.println(toggleShift + "in high gear");
				if(toggleShift){
					System.out.println("SHIFTING LOW TOGGLE!");
					toggleShift = false;
					shiftingTime = Timer.getFPGATimestamp();
					currentDriveState = DriveState.SHIFTING_LOW;
					if(currentSpeedAvg > 0 && Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
						wantedLeftPower = (SHIFTING_POWER * -1);
						wantedRightPower = (SHIFTING_POWER * -1);
					}else if(Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
						wantedLeftPower = (SHIFTING_POWER);
						wantedRightPower = (SHIFTING_POWER);
					}
				}
			}else{
				if(Math.abs(currentSpeedAvg) <= LOWER_SHIFTING_SPEED && shiftStartTime + SHIFT_MIN_BETWEEN < Timer.getFPGATimestamp() || holdFirst || scale){
					System.out.println("SHIFTING LOW!");
					shiftingTime = Timer.getFPGATimestamp();
					shiftStartTime = Timer.getFPGATimestamp();
					currentDriveState = DriveState.SHIFTING_LOW;
					if(currentSpeedAvg > 0 && Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
						wantedLeftPower = (SHIFTING_POWER * -1);
						wantedRightPower = (SHIFTING_POWER * -1);
					}else if(Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER){
						wantedLeftPower = (SHIFTING_POWER);
						wantedRightPower = (SHIFTING_POWER);
					}
				}
			}

			break;
		case SHIFTING_LOW:
			shiftingSol.set(LOW_GEAR);
			if((Math.abs((wantedLeftPower + wantedRightPower) / 2) > SHIFTING_POWER)){
				if(currentSpeedAvg > 0){
					wantedLeftPower = (SHIFTING_POWER * -1);
					wantedRightPower = (SHIFTING_POWER * -1);
				}else{
					wantedLeftPower = (SHIFTING_POWER);
					wantedRightPower = (SHIFTING_POWER);
				}
			}
			if(Timer.getFPGATimestamp() >= shiftingTime + SHIFTING_POWER){
				currentDriveState = DriveState.IN_LOW_GEAR;
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
			// FIXME: Extract 1/8 into constant
			if(!scale){
				wantedAutoSpeed = (AUTO_DRIVE_RAMPING)*(Math.sqrt(Math.abs(wantedAutoDist - currentAutoDist)));
				wantedAutoSpeed = wantedAutoSpeed > .7 ? .7: wantedAutoSpeed;
				wantedAutoSpeed = wantedAutoSpeed < MIN_AUTO_DRIVE_SPEED ? MIN_AUTO_DRIVE_SPEED: wantedAutoSpeed;
				//			LOG.logMessage("wantedAutoDist " + wantedAutoDist);
				//			LOG.logMessage("currentAutoDist " + currentAutoDist);
				//			LOG.logMessage("wantedAutoSpeed " + wantedAutoSpeed);
				//			LOG.logMessage("traveledLeftDistanceAuto " + traveledLeftDistanceAuto);
				//			LOG.logMessage("traveledRightDistanceAuto " + traveledRightDistanceAuto);

				if(Math.abs(traveledLeftDistanceAuto-traveledRightDistanceAuto) < MAX_OFF_DISTANCE_AUTO){
					wantedLeftPower = wantedAutoSpeed;
					wantedRightPower = wantedAutoSpeed;
					//				LOG.logMessage("wantedRightPowerA " + wantedRightPower);
					//				LOG.logMessage("wantedLeftPowerA " + wantedLeftPower);	
				}else if(traveledLeftDistanceAuto < traveledRightDistanceAuto){
					wantedLeftPower = (wantedAutoSpeed * FIX_SPEED_DRIVE_RAMPING) < 1 ? (wantedAutoSpeed * FIX_SPEED_DRIVE_RAMPING): 1;
					wantedRightPower = (wantedAutoSpeed * 0.8) > MIN_AUTO_DRIVE_SPEED ? (wantedAutoSpeed * 0.8): MIN_AUTO_DRIVE_SPEED;
					//				LOG.logMessage("wantedRightPowerB " + wantedRightPower);
					//				LOG.logMessage("wantedLeftPowerB " + wantedLeftPower);
				}else {
					wantedRightPower = (wantedAutoSpeed * FIX_SPEED_DRIVE_RAMPING) < 1 ? (wantedAutoSpeed * FIX_SPEED_DRIVE_RAMPING): 1;
					wantedLeftPower = (wantedAutoSpeed * 0.8) > MIN_AUTO_DRIVE_SPEED ? (wantedAutoSpeed * 0.8): MIN_AUTO_DRIVE_SPEED;
					//				LOG.logMessage("wantedRightPowerC " + wantedRightPower);
					//				LOG.logMessage("wantedLeftPowerC " + wantedLeftPower);
				}


				if(wantedAutoDist < 0){
					wantedRightPower = -wantedRightPower;
					wantedLeftPower = -wantedLeftPower;
				}
			}else{
				if(scalingStartTime + .25 <= Timer.getFPGATimestamp()){
					wantedLeftPower = 1;
					wantedRightPower = 1;
				}else{
					wantedLeftPower = .25;
					wantedRightPower = .25;
				}
			}
			if(Math.abs(currentAutoDist) >= (Math.abs(wantedAutoDist)-6)){
				wantedLeftPower = (STOP_MOTOR);
				wantedRightPower = (STOP_MOTOR);
				//				encoderLeft.reset();
				//				encoderRight.reset();
				//				encoderDataLeft.reset();
				//				encoderDataRight.reset();

				autoState = AutoState.AUTO_STANDBY;
				System.out.println("WE'RE DONE AUTO_DRIVE I HOPE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}
			wantedLeftPower *= -1;
			wantedRightPower *= -1;
			break;

		case AUTO_TURN:
			double currentAngle = angleGyro.getAngle();
			double angleDiff = Math.abs(turnDegreesAuto - currentAngle);
			double speed = angleDiff > 10 ? .75 : .25;//angleDiff > turnDegreesAuto*.2 ? .75 : .15;

			if(currentAngle < turnDegreesAuto){
				wantedRightPower = speed;
				wantedLeftPower = -speed;
			}else{
				wantedRightPower = -speed;
				wantedLeftPower = speed;
			}

			if(currentAngle > (turnDegreesAuto - MAX_TURN_ERROR) && currentAngle < (turnDegreesAuto + MAX_TURN_ERROR)){
				wantedRightPower = STOP_MOTOR;
				wantedLeftPower = STOP_MOTOR;
				autoState = AutoState.AUTO_STANDBY;
				System.out.println("WE'RE DONE AUTO_TURN I HOPE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}
			break;

		case AUTO_DEF:
			switch (defState) {
			case AUTO_REACH_DEF:
				System.out.println("AUTO_REACH_DEF");
				wantedLeftPower = REACH_SPEED;
				wantedRightPower = REACH_SPEED;
				if(tiltGyro.getAngle() > -RAMP_ANGLE){
					defState = AutoState.AUTO_CROSS_DEF;
					System.out.println("Reached the def");
				}
				break;
			case AUTO_CROSS_DEF:
				wantedLeftPower = CROSS_SPEED;
				wantedRightPower = CROSS_SPEED;
				if(tiltGyro.getAngle() < RAMP_ANGLE){
					defState = AutoState.AUTO_COME_DOWN;
					System.out.println("Crossed the def");
				}
				break;
			case AUTO_COME_DOWN:
				wantedLeftPower = COME_DOWN_SPEED;
				wantedRightPower = COME_DOWN_SPEED;
				if(tiltGyro.getAngle() > FLAT_TOL){
					defState = AutoState.AUTO_REACH_DEF;
					autoState = AutoState.AUTO_STANDBY;
					System.out.println("On the other side");
					wantedLeftPower = STOP_MOTOR;
					wantedRightPower = STOP_MOTOR;
				}
			default:
				break;
			}
			break;
		default: System.out.println("Error, auto state is: " + autoState);
		}

		switch (currentScaleState){

		case SCALING_STANDBY:
			break;
		case SCALING_HOOKS: {
			if(scaling.hooked()){
				//ptoSol.set(true);
				//TODO: Set to low gear
				Timer.delay(.15);
				if(wantToScale){
					if(scaleOpControl){
						currentScaleState = ScalingState.MANUAL_SCALING_SCALING;
					}else{
						currentScaleState = ScalingState.SCALING_SCALING;
					}
				}
			}
			break;
		}
		case SCALING_SCALING:
			traveledLeftDistanceScale = Math.abs(encoderDataLeft.getDistance());
			traveledRightDistanceScale = Math.abs(encoderDataRight.getDistance());
			currentScaleDist = (traveledLeftDistanceScale + traveledRightDistanceScale)/2;	
			wantedWinchInPower = (.8/10)*(Math.sqrt(Math.abs(wantedWinchInDistance - currentScaleDist)));
			wantedWinchInPower = wantedWinchInPower > 1 ? 1: wantedWinchInPower;
			wantedWinchInPower = wantedWinchInPower <MIN_SCALE_SPEED ? MIN_SCALE_SPEED: wantedWinchInPower;

			if(Math.abs(currentLeftSpeed-currentRightSpeed) < MAX_SCALE_SPEED_OFF){
				wantedLeftPower = wantedWinchInPower;
				wantedRightPower = wantedWinchInPower;
			}else if(currentLeftSpeed < currentRightSpeed){
				wantedLeftPower = wantedWinchInPower * (FIX_SPEED_SCALE_RAMPING) < 1 ? wantedWinchInPower *(FIX_SPEED_SCALE_RAMPING): 1;
				wantedRightPower = wantedWinchInPower;
			}else {
				wantedRightPower = wantedWinchInPower * (FIX_SPEED_SCALE_RAMPING) < 1 ? wantedWinchInPower *(FIX_SPEED_SCALE_RAMPING): 1;
				wantedLeftPower = wantedWinchInPower;
			}

			if(Math.abs(currentScaleDist) >= Math.abs(wantedWinchInDistance)){
				wantedLeftPower = STOP_MOTOR;
				wantedRightPower = STOP_MOTOR;
				scalingDone = true;
				currentScaleState = ScalingState.SCALING_STANDBY;
			}
			break; 
		case MANUAL_SCALING_SCALING:
			traveledLeftDistanceScale = Math.abs(encoderDataLeft.getDistance());
			traveledRightDistanceScale = Math.abs(encoderDataRight.getDistance());
			currentScaleDist = (traveledLeftDistanceScale + traveledRightDistanceScale)/2;				
			wantedRightPower = wantedWinchInPower;
			wantedLeftPower = wantedWinchInPower;

			// FIXME: Candidate for function - 4
			if(Math.abs(currentLeftSpeed-currentRightSpeed) < MAX_SCALE_SPEED_OFF){
				wantedLeftPower = wantedWinchInPower;
				wantedRightPower = wantedWinchInPower;
			}else if(currentLeftSpeed < currentRightSpeed){
				wantedLeftPower = wantedWinchInPower * (FIX_SPEED_SCALE_RAMPING) < 1 ? wantedWinchInPower *(FIX_SPEED_SCALE_RAMPING): 1;
				wantedRightPower = wantedWinchInPower;
			}else {
				wantedRightPower = wantedWinchInPower * (FIX_SPEED_SCALE_RAMPING) < 1 ? wantedWinchInPower *(FIX_SPEED_SCALE_RAMPING): 1;
				wantedLeftPower = wantedWinchInPower;
			}
			if(Math.abs(currentScaleDist) >= Math.abs(wantedWinchInDistance)){
				wantedLeftPower = STOP_MOTOR;
				wantedRightPower = STOP_MOTOR;
				scalingDone = true;
				currentScaleState = ScalingState.SCALING_STANDBY;
			}

		default: LOG.logError("Were are in this state for scaling: " + currentScaleState);
		break;
		}

		//		if(currentDriveState == DriveState.IN_HIGH_GEAR){
		//			wantedLeftPower = wantedLeftPower > 0 ? Math.pow(wantedLeftPower, 1.1) : -Math.pow(wantedLeftPower, 1.1);
		//			wantedRightPower = wantedRightPower > 0 ? Math.pow(wantedRightPower, 1.1) : -Math.pow(wantedRightPower, 1.1);
		//		}

		if(scale && autoState == AutoState.AUTO_STANDBY){
			wantedLeftPower = wantedWinchInPower;
			wantedRightPower = wantedWinchInPower;
		}

		SmartDashboard.putNumber("Left Drives Speed", currentLeftSpeed);
		SmartDashboard.putNumber("Right Drives Speed", currentRightSpeed);
		leftFront.set(wantedLeftPower);
		leftBack.set(wantedLeftPower);
		rightFront.set(wantedRightPower);
		rightBack.set(wantedRightPower);
		//System.out.println("Left:  " + -encoderDataLeft.getDistance() + "                         " + "Right:  " + encoderDataRight.getDistance());
		//System.out.println("Left A input" + leftA.get() + " Left B input " + leftB.get());
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
		LOG.logMessage("The speeds are (left, right): " + currentLeftSpeed +", " + currentRightSpeed);
		LOG.logMessage("We are currently in this state-------- " + currentDriveState);
		LOG.logMessage("Left:  " + encoderDataLeft.getDistance() + "                         " + "Right:  " + encoderDataRight.getDistance());
		LOG.logMessage("The current winch in distance left is " + (Math.abs(wantedWinchInDistance) - Math.abs(currentScaleDist)));
		LOG.logMessage("We are currently in this Scaling state-------- " + currentScaleState);
		LOG.logMessage("We are currently in this auto state************ " + autoState);
		//		System.out.println("The wanted powers are (left, right): " + wantedLeftPower + ", " + wantedRightPower);
		//		System.out.println("The speeds are (left, right): " + currentLeftSpeed +", " + currentRightSpeed);
		//		System.out.println("Speed Average: " + currentSpeedAvg);
		//		System.out.println("We are currently in this state-------- " + currentDriveState);
		//		System.out.println("We have gone this far!! " + (Math.abs(encoderDataLeft.getDistance()) + Math.abs(encoderDataRight.getDistance()))/2);
		//		System.out.println("The current auto distance left is " + (Math.abs(wantedAutoDist) - Math.abs(currentAutoDist)));
		//		System.out.println("The current winch in distance left is " + (Math.abs(wantedWinchInDistance) - Math.abs(currentScaleDist)));
		//		System.out.println("We are currently in this Scaling state-------- " + currentScaleState);
		//		System.out.println("We are currently in this auto state************ " + autoState);

	}

	/**
	 * is used to get the power from the joysticks 
	 * @param left the left 
	 * joystick input from -1 to 1
	 * @param right the right joystick input from -1 to 1
	 */
	public void setPower(double left, double right) {
		if(currentDriveState != DriveState.SHIFTING_HIGH && currentDriveState != DriveState.SHIFTING_LOW){
			controlsLeftPower = left;
			controlsRightPower = right;
		}
	}
	/**
	 *Makes the states for drives
	 */
	public enum DriveState{
		IN_LOW_GEAR,
		SHIFTING_LOW,
		IN_HIGH_GEAR,
		SHIFTING_HIGH;

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
			default:
				return "Error :(";
			}
		}
	}
	/**
	 *Makes the states for auto
	 */
	public enum AutoState{
		AUTO_DRIVE,
		AUTO_TURN,
		AUTO_STANDBY,
		AUTO_DEF,
		AUTO_REACH_DEF,
		AUTO_CROSS_DEF,
		AUTO_COME_DOWN;

		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case AUTO_DRIVE:
				return "In Auto Drive";
			case AUTO_TURN:
				return "In auto turn";
			case AUTO_STANDBY:
				return "In Auto Standby";
			case AUTO_DEF:
				return "In Auto Def";
			case AUTO_REACH_DEF:
				return "In Auto Reach Def";
			case AUTO_CROSS_DEF:
				return "In Auto Cross Def";
			case AUTO_COME_DOWN:
				return "In Auto Come Down";
			default:
				return "Error :(";
			}
		}
	}
	/**
	 *Makes the states for scaling
	 */
	public enum ScalingState{
		SCALING_STANDBY, 
		SCALING_SCALING,
		SCALING_HOOKS,
		MANUAL_SCALING_SCALING;


		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case SCALING_SCALING:
				return "The scale is scaling";
			case SCALING_STANDBY:
				return "In Scaling standby";
			case SCALING_HOOKS:
				return "The scale is hooking";
			case MANUAL_SCALING_SCALING:
				return "In manual scaling";
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
		encoderLeft.reset();
		encoderRight.reset();
		wantedAutoDist = ((Math.abs(encoderDataLeft.getDistance()) + Math.abs(encoderDataRight.getDistance())) / 2) + length;
		autoState = AutoState.AUTO_DRIVE;
	}

	/**
	 * Called to turn during autonomous
	 * @param angle the angle you want to be (negative for left turn, positive right turn)
	 */
	public void turn(double angle){
		System.out.println("were are going to turn: " + angle);
		angleGyro.reset();
		Timer.delay(.25);
		turnDegreesAuto = angle;
		autoState = AutoState.AUTO_TURN;
	}

	/**
	 * return true if the auto function finished
	 */
	public boolean autoFunctionDone(){
		return autoState == AutoState.AUTO_STANDBY;
	}

	/**
	 * stops everything in drives and puts auto in standby
	 */
	public void autoEStop(){
		autoState = AutoState.AUTO_STANDBY;
		currentDriveState = DriveState.IN_LOW_GEAR;
		currentScaleState = ScalingState.SCALING_STANDBY;
		wantedLeftPower = STOP_MOTOR;
		wantedRightPower = STOP_MOTOR;
	}

	/**
	 * called to set the auto state to auto defense
	 */
	public void startAutoDef(){
		autoState = AutoState.AUTO_DEF;
		defState = AutoState.AUTO_REACH_DEF;
		tiltGyro.reset();
		System.out.println("AUTO DEFFFFFFFFFFFFFFFFFFFFFF");
	}

	/**
	 * Called by Scaling when arms have been extended and we want to winch out 
	 * @param distanceToScale= the distance we need to scale
	 * @param winchInPower= the power to winch in
	 */
	public void scaleWinch(double distanceToScale) {
		currentScaleState = ScalingState.SCALING_HOOKS;
		wantedWinchInDistance = distanceToScale;
		engagePto = true;
	}

	/**
	 * Checks to see if scaling is done 
	 * @return
	 */
	public boolean isScaleScalingDone() {
		if(scalingDone){
			scalingDone = false;
			return !scalingDone;
		}
		return false;
	}

	/**
	 * if called, lets the driver manually shift
	 */
	public void driverShifting(){
		driverShift = !driverShift;
	}

	/**
	 * called to manually shift up or down
	 */
	public void toggleShifting(){
		toggleShift = true;
	}

	/**
	 * If called, will either engage or disengage the pto depending on it's previous state, toggle on off
	 */
	public void manualPtoEngage(){
		engagePto = !engagePto;
		scaleOpControl = !engagePto;
		currentScaleState = scaleOpControl == false ? ScalingState.SCALING_STANDBY :ScalingState.SCALING_HOOKS;
	}

	/**
	 * Called to manually scale with the joystick
	 * @param power the power that the joystick is giving 
	 */
	public void manualScale(double power){
		wantedLeftPower = power;
		wantedRightPower = power;
	}

	/**
	 * called to be able to start scaling
	 */
	public void beginScaling(){
		//youre cute :)
		wantToScale = !wantToScale;
	}

	/**
	 * called to emergency stop the scaling, will not retract the winch
	 */
	public void eStopScaling(){
		wantedWinchInPower = STOP_MOTOR;
		//ptoSol.set(false);
		currentScaleState = ScalingState.SCALING_STANDBY;
	}

	public void holdFirst(boolean newVal){
		holdFirst = newVal;
	}

	public void killAutoDrive(){
		autoState = AutoState.AUTO_STANDBY;
	}

	public void returnToZero(){
		if(Math.abs(angleGyro.getAngle()) > 5){
			turnDegreesAuto = -angleGyro.getAngle();
			angleGyro.reset();
			Timer.delay(.25);
			autoState = AutoState.AUTO_TURN;
		}
	}

	public void scale(){
		wantedAutoDist = 36 * 6;
		scale = true;
		scalingStartTime = Timer.getFPGATimestamp(); 
		autoState = AutoState.AUTO_DRIVE;
	}
}