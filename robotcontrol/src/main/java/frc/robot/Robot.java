package frc.robot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
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

	

	//pneumatic control
	Compressor c = new Compressor(0);
	DoubleSolenoid DoubleSole = new DoubleSolenoid(0, 1);
	boolean triggerStatus;
	
	@Override
	public void robotInit() {
		NetworkTableInstance inst =NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("SmartDashboard");
		NetworkTableEntry xEntry=table.getEntry("x");	

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
	boolean turnMode = false;
	boolean ServoStatus = false;
	boolean ServoButtonState = false;
	boolean VisionStatus = false;
	boolean visionMode = false;
	boolean VcorrectStatus = false;
	double Iangle = 0;
	double addTurn = 0;
	double Rdesire = -90;
	double Fdesire = 90;

	// left and right 90 turn variable
	boolean Lstatus = false;
	boolean LMode = false;
	boolean RMode = false;
	boolean Rstatus = false;
	boolean Ladjust = false;
	boolean Radjust = false;



	@Override
	public void teleopPeriodic() {
		double VisionCord = xEntry.getDouble(0.0);
		System.out.println("This is VisionCord:" + VisionCord);

		// Gamepad processing	
		boolean Right90 = _gamepad.getRawButton(10);
		boolean Left90 = _gamepad.getRawButton(9);
		boolean TurnButton = _gamepad.getRawButton(3); 
		boolean encoderReset = _gamepad.getRawButton(7);
		boolean TurnOff = _gamepad.getRawButton(12);
		boolean compressorButton = _gamepad.getRawButton(5);
		boolean VisionButton = _gamepad.getRawButton(6);
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		boolean trigger = _gamepad.getTrigger();
		boolean toptrigger = _gamepad.getTop();
		double sensitivity =1-( _gamepad.getThrottle() + 1)/2;
		boolean turn90 = _gamepad.getRawButton(4);
		
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
	
		
		double adjustSensitivity = 0.4;
	/*	



		if (visionButton == true) {	
			System.out.println("buttonpressed");
			if (right && !left) {
				System.out.println("right & not left");
				visionCorrectAmt = adjustSensitivity;
			}else if (left && !right) {
				System.out.println("left & not right");
				visionCorrectAmt = -adjustSensitivity;
			}else if (left && right) {
				System.out.println("left & right");
				visionCorrectAmt = 0;
			}else {
				System.out.println("not left & not right");
				visionCorrectAmt = 0;
			}
		}else {
			visionCorrectAmt = 0;
		}
*/
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
		
		
	
		/*

		if(turnMode == true){
			if(ahrs.getYaw()- Iangle <95){
				addTurn = 0.5;
			}else if (ahrs.getYaw() - Iangle > -95){
				addTurn = -0.5;
			
			}
		}
		if(TurnOff == true){
			turnMode = false;
			addTurn = 0;
		}*/

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
		if (c.getPressureSwitchValue()==true){
			c.setClosedLoopControl(false);
		}else if(c.getPressureSwitchValue()==false && compressorStatus == true){
			c.setClosedLoopControl(true);
		}




		if (VisionButton == true && VisionStatus == false) {
			if(visionMode){
				VisionStatus = false;
				VcorrectStatus = true;
			} else if (VisionStatus == false) {
				VisionStatus = true;
				VcorrectStatus = false;
			}
		    visionMode = true;
		}else if (VisionButton == false && VisionStatus == true) {
			visionMode = false;
		}

		if (VcorrectStatus = true && xEntry.getDouble(0.0) - 160 > 10 && xEntry.getDouble(0.0) - 160 < -10){
			VisionPID();

		}
		if (xEntry.getDouble(0.0) == 0 && xEntry.getDouble(0.0) == 160 && xEntry.getDouble(0.0) == 320){
			visionCorrectAmt = 0;
		}
		if (VcorrectStatus = false){
			visionCorrectAmt = 0;
		}

		if (Left90 == true && Lstatus == false) {
			Rstatus = false;
			RMode = false;
			ahrs.reset();
			if(LMode){
				Lstatus = false;
				Ladjust = true;
			} else if (Lstatus == false) {
				Lstatus = true;
				
				Ladjust = false;
			}
		    LMode = true;
		}else if (Left90 == false && Lstatus == true) {
			LMode = false;
		}

		if (Ladjust == true){
			Lturn90();
		}
		if (Ladjust == false){
			Lturn = 0;
		}

		if (Right90 == true && Rstatus == false) {
			Lstatus = false;
			LMode = false;
			ahrs.reset();
			if(RMode){
				Rstatus = false;
				Radjust = true;
			} else if (Rstatus == false) {
				Rstatus = true;
				
				Radjust = false;
			}
		    RMode = true;
		}else if (Right90 == false && Rstatus == true) {
			RMode = false;
		}

		if (Radjust == true){
			Rturn90();
		}
		if (Radjust == false){
			Rturn = 0;
		}

		System.out.println(distance);
		System.out.println(distance1);
		if (encoderReset) {
			EncLeft.reset();
			EncRight.reset();
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
		_drive.arcadeDrive(turn+rcw+visionCorrectAmt, forward);
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
	public void VisionPID(){
		Vdifference = xEntry.getDouble(0.0) - 160;
		this.pre_Verror = Vdifference;
		if (Vdifference < 80 && Vdifference > -80){
			this.Vintegral +=(Vdifference*0.02);
		} else {
			this.Vintegral = 0;
		}
		Vderivative = (Vdifference - this.previous_error)/0.02;
		if (Vdifference > 100 || Vdifference < -100){
			this.visionCorrectAmt = 0.7;
		}else if(Vdifference < 100 && Vdifference > -100){
			this.visionCorrectAmt = Vp*Vdifference + Vi*Vintegral + Vd*Vdifference;
		}



	}
	public void Rturn90(){
		Rdifference = Rdesire - ahrs.getYaw();
		if (Rdifference < 40) {
			this.Rintegral +=(Rdifference *0.02);

		}else{
			this.Rintegral = 0;
		}
		this.Rp_error = Rdifference;
		Rderivative = (Rdifference - Rp_error);
		if (Rdifference > 50){
			this.Rturn = 0.7;
		}else if(Rdifference < 50){
			this.Rturn = Rdifference*Tp + Rintegral*Ti + Rderivative*Td;
		}
		
		
	}
	public void Lturn90(){
		Ldifference = Fdesire - ahrs.getYaw();
		if (Ldifference < 40) {
			this.Lintegral +=(Ldifference *0.02);

		}else{
			this.Lintegral = 0;
		}
		this.Lp_error = Ldifference;
		Lderivative = (Ldifference - Lp_error);
		if (Ldifference > 50){
			this.Lturn = 0.7;
		}else if(Ldifference < 50){
			this.Lturn = Ldifference*Tp + Lintegral*Ti + Lderivative*Td;
		}

	}
	


	
	
}
