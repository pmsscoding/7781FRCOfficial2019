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
import edu.wpi.first.wpilibj.DoubleSolenoid;


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
		System.out.println("drive");
		setangle = ahrs.getYaw();
		c.setClosedLoopControl(false);
	}
	
	boolean Mode = false;
	boolean compressorstatus = false;
	@Override
	public void teleopPeriodic() {
		NetworkTableInstance inst =NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("Shuffleboard");
		xEntry=table.getEntry("yeet");		
		
		// Gamepad processing	
		boolean valveOpen = _gamepad.getRawButton(5);
		boolean valveClose = _gamepad.getRawButton(3);
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		boolean trigger = _gamepad.getTrigger();
		boolean toptrigger = _gamepad.getTop();
		double sensitivity =1-( _gamepad.getThrottle() + 1)/2;
		boolean valveOff = _gamepad.getRawButtonPressed(4);
		
		// status
		boolean autoadjust = false;

		//solenoid control
		if (valveOpen) {
			DoubleSole.set(DoubleSolenoid.Value.kReverse); 
			System.out.println("valve open");
		}
		if (valveClose) {
			DoubleSole.set(DoubleSolenoid.Value.kForward);
			System.out.println("valve close");
		}
		if (valveOff) {
			DoubleSole.set(DoubleSolenoid.Value.kOff);
		}

		// servo arm control
		_rightServo.setAngle(0);
		_leftServo.setAngle(0);

		// compressor control
		if (trigger == true && Mode == false) { //1 press for on off
			if (compressorstatus) {
				compressorstatus = false;
				c.setClosedLoopControl(false);
			} else if (compressorstatus == false) {
				compressorstatus = true;
				c.setClosedLoopControl(true);
			}
			Mode = true;
		}else if (trigger == false && Mode == true) {
			Mode = false;
		}
		if (c.getPressureSwitchValue()){
			c.setClosedLoopControl(false);
		}
		
		/*driving logic*/
		forward = Deadband(forward);
		turn = Deadband(turn);
		forward *= sensitivity;
		turn *= sensitivity;

		//gyro pid processing
		if (toptrigger){
			if(autoadjust == true) {
				autoadjust = false;
			}
			if(autoadjust == false) {
				autoadjust = true;
			}
		}
		if (autoadjust){
			GyroPID();
		}
		if (autoadjust == false){
			setangle = ahrs.getYaw();
			rcw = 0;
		}
		_drive.arcadeDrive(turn+rcw, forward);
	
	// Deadband 5 percent, used on the gamepad
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.075) { return value; }
		
		/* Lower deadband */
		if (value <= -0.075) { return value; }
		
		/* Outside deadband */
		return 0;
	}
	
	public void GyroPID(){
		double difference = setangle - ahrs.getYaw();
		this.previous_error = difference;
		if (difference < 60 && difference > -60){
			this.integral +=(difference*0.02);
		} else {
			this.integral = 0;
		}
		derivative = (difference - this.previous_error)/0.02;
		if (difference > 90 || difference < -90) {
			this.rcw = 0.7;
		 }else {
			this.rcw = P*difference + I*this.integral - D*derivative;
		}
	}
}
