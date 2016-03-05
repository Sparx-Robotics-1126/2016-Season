package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.subsystem.BallAcqNew;
import org.gosparx.team1126.robot.util.AdvancedJoystick;
import org.gosparx.team1126.robot.util.AdvancedJoystick.ButtonEvent;
import org.gosparx.team1126.robot.util.AdvancedJoystick.JoystickListener;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A class for controlling the inputs from controls.
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class Controls extends GenericSubsystem implements JoystickListener{

	/**
	 * the input from the left joystick
	 */
	private double leftPower;

	/**
	 * the input from the right joystick
	 */
	private double rightPower;

	/**
	 * the deadband on the joystick of which we don't want it to move
	 */
	private static final double DEADBAND = 0.05;

	/**
	 * used to check if we want to manually control the pto
	 */
	private boolean manualPto = false;


	/**
	 * the instance of the camera controller
	 */
	private static CameraController camCont;

	/**
	 * The instance of driver station
	 */
	private static DriverStation ds;

	/**
	 * Support for singleton
	 */
	private static Controls controls;

	/**
	 * The advanced joystick for the right driver stick
	 */
	private AdvancedJoystick driverRight;

	/**
	 * The advanced joystick for the left driver stick
	 */
	private AdvancedJoystick driverLeft;

	/**
	 * The advanced joystick for the operator
	 */
	private AdvancedJoystick opJoy;

	/**
	 * declares a BallAcq named ballAcq
	 */
	private BallAcqNew ballAcq;
	
	private boolean opControl;
	private boolean opControlPrev;
	
	private double drawbridgeStart;
	private static final double DRAWBRIDGE_TIME = .25;

	//xbox mapping
	private static final int XBOX_A = 1;
	private static final int XBOX_B = 2;
	private static final int XBOX_X = 3;
	private static final int XBOX_Y = 4;
	private static final int XBOX_L1 = 5;
	private static final int XBOX_R1 = 6;
	private static final int XBOX_BACK = 7;
	private static final int XBOX_START = 8;
	private static final int XBOX_L3 = 9;
	private static final int XBOX_R3 = 10;
	private static final int XBOX_LEFT_X = 0;
	private static final int XBOX_LEFT_Y = 1;
	private static final int XBOX_L2 = 2;
	private static final int XBOX_R2 = 3;
	private static final int XBOX_RIGHT_X = 4;
	private static final int XBOX_RIGHT_Y = 5;
	private static final int XBOX_POV = 0;

	/**
	 * The outputs for the joysticks 
	 */
	private static final int NEW_JOY_X_AXIS = 0;
	private static final int NEW_JOY_Y_AXIS = 1;
	private static final int NEW_JOY_TRIGGER = 1;//TRIGGEr
	private static final int NEW_JOY_LEFT = 2;//LEFT
	private static final int NEW_JOY_RIGHT = 3;//RIGHT
	private static final int NEW_JOY_MIDDLE = 4;

	private int lastPOV;

	/**
	 * @return the only instance of Controls ever.
	 */
	public static synchronized Controls getInstance(){
		if(controls == null){
			controls = new Controls();
		}
		return controls;
	}

	/**
	 * Creates a new controls
	 */
	private Controls() {
		super("Controls", Thread.MAX_PRIORITY);
	}

	/**
	 * Sets everything up.
	 */
	@Override
	protected boolean init() {
		driverLeft = new AdvancedJoystick("Driver Left", IO.DRIVER_JOY_LEFT,4,DEADBAND);
		driverLeft.addActionListener(this);
		driverLeft.addButton(NEW_JOY_LEFT);
		driverLeft.addButton(NEW_JOY_TRIGGER);
		driverLeft.addButton(NEW_JOY_RIGHT);
		driverLeft.start();

		driverRight = new AdvancedJoystick("Driver Right", IO.DRIVER_JOY_RIGHT,4,DEADBAND);
		driverRight.addActionListener(this);
		driverRight.addButton(NEW_JOY_LEFT);
		driverRight.addButton(NEW_JOY_TRIGGER);
		driverRight.addButton(NEW_JOY_RIGHT);
		driverRight.start();

		opJoy = new AdvancedJoystick("Operator Joy", IO.USB_OPERATOR, 10, 0.25);
		opJoy.addActionListener(this);
		opJoy.addButton(XBOX_Y);
		opJoy.addButton(XBOX_R1);
		opJoy.addButton(XBOX_BACK);
		opJoy.addButton(XBOX_L1);
		opJoy.addButton(XBOX_L2);
		opJoy.addButton(XBOX_START);
		opJoy.addButton(XBOX_B);
		opJoy.addButton(XBOX_A);
		opJoy.addButton(XBOX_X);
		opJoy.addButton(XBOX_Y);
		opJoy.start();

		leftPower = 0;
		rightPower = 0;
		opControl = false;
		opControlPrev = false;
		ds = DriverStation.getInstance();
		ballAcq = BallAcqNew.getInstance();
		camCont = CameraController.getInstance();

		return true;
	}

	/**
	 * Pointless in this class
	 */
	@Override
	protected void liveWindow() {

	}

	/**
	 * Loops, controls drives
	 */
	@Override
	protected boolean execute() {
		if(ds.isOperatorControl()){
			leftPower = driverLeft.getAxis(NEW_JOY_Y_AXIS);
			rightPower = driverRight.getAxis(NEW_JOY_Y_AXIS);
			if(opJoy.getPOV(XBOX_POV) == 0 && lastPOV != 0){
				LOG.logMessage("OP Button: Home with Rollers");
				ballAcq.homeRollers();
			}else if(opJoy.getPOV(XBOX_POV) == 180 && lastPOV != 180){
				LOG.logMessage("OP Button: Go to floor");
				ballAcq.goToLowBarPosition();
			}else if(opJoy.getPOV(XBOX_POV) == 270 && lastPOV != 270){
				LOG.logMessage("OP Button: Home without rollers");
				ballAcq.setHome();
			}else if(opJoy.getPOV(XBOX_POV) == 90 && lastPOV != 90){
				LOG.logMessage("OP Button: At Acquire Ball Position");
				ballAcq.acquireBall();
			}
			
			if(opJoy.getAxis(XBOX_R2) > .5){
				System.out.println("we're firing!");
				ballAcq.fire();
			}
			
			opControl = opJoy.getAxis(XBOX_RIGHT_Y) != 0;
			
			if(opControl != opControlPrev){
				ballAcq.setOpControl(opControl);
			}
			if(opControl){
				ballAcq.setArmPower(Math.pow(opJoy.getAxis(XBOX_RIGHT_Y), 3));
			}
			
			if(Timer.getFPGATimestamp() > drawbridgeStart + DRAWBRIDGE_TIME){
				ballAcq.goToLowBarPosition();
				drawbridgeStart = Double.MAX_VALUE;
			}
			
			opControlPrev = opControl;
			lastPOV = (int) opJoy.getPOV(XBOX_POV);
		}
		return false;
	}

	/**
	 * Small sleeps for accurate control
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * Writes info to a log every 5 seconds.
	 */
	@Override
	protected void writeLog() {

	}

	public void actionPerformed(ButtonEvent e) {
		if(ds.isOperatorControl()){
			switch(e.getPort()){
			case IO.USB_OPERATOR:
				switch(e.getID()){
				case XBOX_A:
					//Toggle Rollers
					if(e.isRising()){
						ballAcq.toggleRoller();
						LOG.logMessage("OP Button: Toggle Roller");
					}
					break;
				case XBOX_B:
					//Reverse Rollers
					if(e.isRising()){
						ballAcq.reverseRoller();
						LOG.logMessage("OP Button: Reverse Rollers");
					}
					break;
				case XBOX_Y:
					//Stop All
					if(e.isRising()){
						ballAcq.stopAll();
						LOG.logMessage("OP Button: E-Stop ballAcq");
					}
					break;
				case XBOX_X:
					//DRAWBRIDGE
					if(e.isRising()){
						drawbridgeStart = Timer.getFPGATimestamp();
						ballAcq.setHome();
						LOG.logMessage("OP Button: Draw bridge");
					}
					break;
				default:
					LOG.logMessage("Bad button id" + e.getID());
				}
			case IO.DRIVER_JOY_LEFT:
				switch(e.getID()){
				case NEW_JOY_TRIGGER:
					if(e.isRising()){
					  //camCont.switchCamera();
					  //System.out.println("Toggle Camera");
						//System.out.println("Started Auto Drive");
						//drives.driveWantedDistance(50);
					}
					break;
				case NEW_JOY_LEFT:
					if(e.isRising()){
						//drives.startAutoDef();
						System.out.println("Toggle Shifting");
					}
					break;
				case NEW_JOY_RIGHT:
					if(e.isRising()){
						System.out.println("Driver wants to shift");
					}else 
						System.out.println(e.isRising());
				}
				break;
			case IO.DRIVER_JOY_RIGHT:
				switch(e.getID()){
				case NEW_JOY_TRIGGER:
					manualPto = !manualPto;
					break;
				case NEW_JOY_LEFT:
					break;
				case NEW_JOY_RIGHT:
					//andrews method in scaling
					break;
				}
				break;
			}
		}
	}
}


