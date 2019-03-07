package frc.robot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
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
	
	//networktable
	NetworkTableEntry xEntry;

	//encoder
	SerialPort serial = new SerialPort(123, SerialPort.Port.kOnboard);
	Encoder EncRight = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder EncLeft = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
	SerialPort ser = new SerialPort(9600, SerialPort.Port.kMXP);
	DigitalInput leftTurn = new DigitalInput(4);
	DigitalInput rightTurn = new DigitalInput(5);

	//GYRO and PID setting
	double setangle = 0;
	double integral, previous_error, rcw, derivative, pre_Verror, Vintegral, Vderivative, Vdifference = 0;
	double Rdifference, Ldifference, Rintegral, Lintegral, Rderivative, Lderivative = 0;
	double P = 0.02;
	double I = 0.05;
	double D = 0.05;
	double Vp = 0.02;
	double Vi = 0.05;
	double Vd = 0.05;
	double Tp = 0.04;
	double Ti = 0.1;
	double Td = 0.1;
	double Lp_error = 0;
	double Rp_error = 0;
	double Rturn = 0;
	double Lturn = 0;
	double visionCorrectAmt = 0;
	double SetTurn = 0;
	AHRS ahrs;
	//Networktable variables
	NetworkTableEntry xEntry;

	//pneumatic control
	Compressor c = new Compressor(0);
	DoubleSolenoid DoubleSole = new DoubleSolenoid(0, 1);
	boolean triggerStatus;
	
	@Override
	public void robotInit() {
	NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");
	xEntry=table.getEntry("x");
	
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
		_rightServo.setAngle(0);
		_leftServo.setAngle(0);
	}
	
	boolean compresserButtonState = false;
	boolean compressorStatus = false;
	boolean pistonButtonState = false;
	boolean pistonStatus = false;
	float hatchAngle;
	float rocketAngleRight;

	@Override
	public void teleopPeriodic() {
		//network table
		double x= xEntry.getDouble(0.0);
    	System.out.println("This is x:" + x);

		// Gamepad processing	
		boolean Right90 = _gamepad.getRawButton(4);
		boolean Left90 = _gamepad.getRawButton(6);
		boolean toptrigger = _gamepad.getRawButton(11); 
		boolean encoderReset = _gamepad.getRawButton(7);
		
		boolean compressorButton = _gamepad.getRawButton(5);
		boolean VisionButton = _gamepad.getRawButton(3);
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		boolean trigger = _gamepad.getTrigger();
		boolean toptrigger = _gamepad.getTop();
		boolean angleButton = _gamepad.getRawButton(12);
		boolean rocketLeftButton = _gamepad.getRawButton(9);
		boolean rocketRightButton = _gamepad.getRawButton(10);
		double sensitivity =1-( _gamepad.getThrottle() + 1)/2;
		
		double distance = EncLeft.getRaw();
		double distance1 = EncRight.getRaw();

		// status
		boolean autoadjust = false;

		// servo arm control
		
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
		
		//visionstuff
		if (visionButton == true) {	
			if (xEntry.getDouble(0.0) - 160 > 10) {
				visionCorrectAmt = 0.4;
			}
			else if (xEntry.getDouble(0.0) - 160 < -10) {
				visionCorrectAmt = -0.4;
			}
			else {
				visionCorrectAmt = 0;
			}
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

		if (TurnButton == true && ServoButtonState == false) { //1 press for on off
			if (ServoStatus) {
				ServoStatus = false;
				_rightServo.setAngle(0);
				_leftServo.setAngle(0);
			} else if (ServoStatus == false) {
				ServoStatus = true;
				
				_rightServo.setAngle(90);
				_leftServo.setAngle(90);
			}
			ServoButtonState = true;
		}else if (TurnButton == false && ServoButtonState == true) {
			ServoButtonState = false;
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