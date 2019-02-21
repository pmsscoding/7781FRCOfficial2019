package frc.robot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.IMUProtocol.GyroUpdate;
import com.kauailabs.navx.frc.AHRS;

import com.kauailabs.navx.frc.*;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;

public class Robot extends TimedRobot {
	/** Hardware, either Talon could be a Victor */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(3);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(6);
	Joystick _gamepad = new Joystick(0);
	Servo _leftServo = new Servo(0);
	Servo _rightServo = new Servo(1);
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(4);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(5);
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);
	
	//encoder
	SerialPort serial = new SerialPort(123, SerialPort.Port.kOnboard);
	Encoder EncRight = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder EncLeft = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
	
	DigitalInput leftTurn = new DigitalInput(4);
	DigitalInput rightTurn = new DigitalInput(5);

	//GYRO and PID setting
	double setangle = 0;
	double integral, previous_error, rcw, derivative = 0;
	double P = 0.02;
	double I = 0.05;
	double D = 0.05;
	AHRS ahrs;
	//Networktable variables
	NetworkTableEntry xEntry;
	NetworkTableEntry yEntry;
	double yeet;

	//pneumatic control
	Compressor c = new Compressor(0);
	DoubleSolenoid DoubleSole = new DoubleSolenoid(0, 1);
	boolean triggerStatus;
	
	@Override
	public void robotInit() {
    		CameraServer.getInstance().startAutomaticCapture();
		try {
			ahrs = new AHRS(Port.kUSB1);
		} catch(RuntimeException ex) {
			System.out.println("navx gyro error");
		}
		EncLeft.reset();
		EncRight.reset();
	}
	@Override
	public void teleopInit(){
		/* Ensure motor output is neutral during init */
		_frontLeftMotor.configFactoryDefault();
		_frontRightMotor.configFactoryDefault();
		_leftSlave1.configFactoryDefault();
		_rightSlave1.configFactoryDefault();
		
		/*follow other motor*/
		_leftSlave1.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		
		/* Set Neutral mode */
		_frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		_frontRightMotor.setNeutralMode(NeutralMode.Brake);
		
		/* Configure output direction */
		_frontLeftMotor.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
		_frontRightMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
		_leftSlave1.setInverted(InvertType.FollowMaster);
		_rightSlave1.setInverted(InvertType.FollowMaster);
		setangle = ahrs.getYaw();
		c.setClosedLoopControl(false);
	}
	
	boolean compresserButtonState = false;
	boolean compressorStatus = false;
	boolean pistonButtonState = false;
	boolean armButtonState = false;
	boolean pistonStatus = false;
	float hatchAngle;
	float rocketAngleRight;

	@Override
	public void teleopPeriodic() {
		NetworkTableInstance inst =NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("Shuffleboard");
		xEntry=table.getEntry("yeet");		

		// Gamepad processing	
		boolean compressorButton = _gamepad.getRawButton(5);
		boolean armButton = _gamepad.getRawButton(4);
		boolean encoderReset = _gamepad.getRawButton(3);
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		boolean trigger = _gamepad.getTrigger();
		boolean toptrigger = _gamepad.getTop();
		boolean angleButton = _gamepad.getRawButton(12);
		boolean rocketLeftButton = _gamepad.getRawButton(9);
		boolean rocketRightButton = _gamepad.getRawButton(10);
		double sensitivity =1-( _gamepad.getThrottle() + 1)/2;
		boolean visionButton = _gamepad.getRawButton(6);
		
		double distance = EncLeft.getRaw();
		double distance1 = EncRight.getRaw();

		// status
		boolean autoadjust = false;

		/*
		How buttonState works:
		getRawButton() will periodically produce true/false
		to reject subsequent "true"s when the button is pressed for a longer time,
		the button will set a buttonState.
		when buttonState and button are misaligned (eg one is false one is true) that means
		a the button has just been pressed or released, and will realign the two.
		using this we can detect the first time the button is triggered and released.
		*/
		// vision targeting control		
		boolean left = leftTurn.get();
		boolean right = rightTurn.get();

		double visionCorrectAmt = 0;
		double adjustSensitivity = 0.4;

		if (visionButton == true) {	
			System.out.println("buttonpressed");
			/*if (right == true && left == false) {
				System.out.println("right & not left, turning right");
				visionCorrectAmt = adjustSensitivity;
			}
			else if (left == true && right == false) {
				System.out.println("left & not right, turning left");
				visionCorrectAmt = -adjustSensitivity;
			}
			else if (left == false && right == false) {
				System.out.println("left & right, object detected");
				visionCorrectAmt = 0;
			}else {
				System.out.println("else block");
				visionCorrectAmt = 0;
			}*/
			System.out.println(left);
			System.out.println(right);

		}else {
			visionCorrectAmt = 0;
		}

		// compressor control
		if (compressorButton == true && compresserButtonState == false) { //1 press for on off
			if (compressorStatus) {
				compressorStatus = false;
				c.setClosedLoopControl(false);
			} else if (compressorStatus == false) {
				compressorStatus = true;
				c.setClosedLoopControl(true);
			}
			compresserButtonState = true;
		}else if (compressorButton == false && compresserButtonState == true) {
			compresserButtonState = false;
		}

		//piston extend and retract
		if (trigger == true && pistonButtonState == false) {
			if(pistonStatus){
				pistonStatus = false;
				DoubleSole.set(DoubleSolenoid.Value.kReverse);
			} else if (pistonStatus == false) {
				pistonStatus = true;
				DoubleSole.set(DoubleSolenoid.Value.kForward);
			}
			pistonButtonState = true;
		}else if (trigger == false && pistonButtonState == true) {
			pistonButtonState = false;
		}

		//automatic compressor succ
		if (c.getPressureSwitchValue()==true){
			c.setClosedLoopControl(false);
		}else if(c.getPressureSwitchValue()==false && compressorStatus == true){
			c.setClosedLoopControl(true);
		}
		
		if (encoderReset) {
			EncLeft.reset();
			EncRight.reset();
		}
		/*driving logic*/
		forward = Deadband(forward);
		turn = Deadband(turn);
		forward *= sensitivity;
		turn *= sensitivity;
		double rocketAngleLeft = hatchAngle+90;
		double RawAngle = ahrs.getYaw();
		double currentAngle = RawAngle + 180;
		double error = Math.abs(rocketAngleLeft) - Math.abs(currentAngle);
		double turnAdd=0;
		//encoder variables

		//encoder x and y stuff
		double xDisplacement = ahrs.getDisplacementY();
		double yDisplacement = ahrs.getDisplacementX();

		//System.out.println("y displacement" + yDisplacement);
		//System.out.println("x displacement" + xDisplacement);
		//robot general align to rocket

		if (angleButton) {
			hatchAngle = ahrs.getYaw(); //logs "master angle" to find other angles on game field
			System.out.println(currentAngle); //print angle
			System.out.println(RawAngle);
		}
		if (rocketRightButton) {//rocket right angle
			if (Math.abs(error)>=40) {
				turnAdd = -0.5;
				System.out.println("turning to angle" + error);
			}
			else if (Math.abs(error)<40){
				if (currentAngle<rocketAngleLeft) {
					turnAdd = 0.43;
					if (error<10) {
						turnAdd = 0;
						System.out.println("Angle Reached");
					}
				}
				else if (currentAngle>=rocketAngleLeft) {
					turnAdd = -0.43;
					if (error<10) {
						turnAdd = 0;
						System.out.println("Angle Reached");
					}
				}
				turnAdd = 0.43;
				System.out.println("precise mode" + error);
			}
		}

		//arm control
		if (armButton == true) {
			_rightServo.setAngle(135);
			_leftServo.setAngle(135);
		}
		else if (armButton == false) {
			_rightServo.setAngle(45);
			_leftServo.setAngle(45);
		}

		//gyro pid processing
		_drive.arcadeDrive(turn+rcw+visionCorrectAmt+turnAdd, forward);
	}
	// Deadband 5 percent, used on the gamepad
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.075) { return value; }
		
		/* Lower deadband */
		if (value <= -0.075) { return value; }
		
		/* Outside deadband */
		return 0;
	}
}