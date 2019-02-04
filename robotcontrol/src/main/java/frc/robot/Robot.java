package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

public class Robot extends TimedRobot {
	/** Hardware, either Talon could be a Victor */
	//VictorSPX _leftMaster = new VictorSPX(4);
	//VictorSPX _rightMaster = new VictorSPX(5);
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(3);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(6);
	Joystick _gamepad = new Joystick(0);
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(4);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(5);
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);
	
	private static final int kUltrasonicPort = 0;
	private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
	
	//GYRO and PID setting
	double setangle = 0;
	double integral, previous_error, rcw, derivative = 0;
	double P = 0.02;
	double I = 0.05;
	double D = 0.05;
	AHRS ahrs;

	//networktablestuff
	NetworkTableEntry x;
	
	

	@Override
	public void robotInit() {

		NetworkTableInstance inst=NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("SmartDashBoard");
		x=table.getEntry("yeet");
    		CameraServer.getInstance().startAutomaticCapture();
		try {
			ahrs = new AHRS(Port.kUSB1);
		} catch(RuntimeException ex) {
			System.out.println("navx gyro error");
		}
		/* Not used in this project */
	}
	
	@Override
	public void teleopInit(){
		/* Ensure motor output is neutral during init */
		_frontLeftMotor.configFactoryDefault();
		_frontRightMotor.configFactoryDefault();
		_leftSlave1.configFactoryDefault();
		_rightSlave1.configFactoryDefault();

		/* Factory Default all hardware to prevent unexpected behaviour */
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
	}
	
	@Override
	public void teleopPeriodic() {
		double xD=x.getDouble(-1);
		System.out.println(xD);		
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		boolean trigger = _gamepad.getTrigger();
		double sensitivity =1-( _gamepad.getThrottle() + 1)/2;
		System.out.println(sensitivity);

		
		//gyro pid processing
		GyroPID();
		
		boolean angletrigger = _gamepad.getTop();
		if (angletrigger) {
			System.out.println(ahrs.getYaw());
			setangle = ahrs.getYaw();
		}
			
		forward = Deadband(forward);
		turn = Deadband(turn);
		

		if (trigger) {
			if (forward>0.45|turn>0.45) {
				while (forward>0.45) {
					forward = forward*0.9;
				}
				while (turn>0.45){
					turn = turn*0.9;
				}
			}
			if (forward<0.45 & turn<0.45){
				forward *= sensitivity;
				turn *= sensitivity;
			}
		}
		
		_drive.arcadeDrive(turn, forward);
	    }
	

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.075) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.075)
			return value;
		
		/* Outside deadband */
		return 0;
	}
	public void GyroPID(){

		double difference = setangle - ahrs.getYaw();
		
		this.previous_error = difference;
		if (difference < 60 && difference > -60){
			this.integral +=(difference*0.02);

		}else{
			this.integral = 0;

		}
		derivative = (difference - this.previous_error)/0.02;
		this.rcw = P*difference + I*this.integral - D*derivative;
	}
}