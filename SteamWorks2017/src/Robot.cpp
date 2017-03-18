#include <iostream>
#include <iomanip>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"
#include <CANTalon.h>
//#include <LiveWindow/LiveWindow.h>
//#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot {
private:


	//Feeder and shooter motor
	Servo *gearServoRight;
	Servo *gearServoLeft;
	TalonSRX *feederMotor;
	CANTalon *shooter;
	double speed;
	int timer = 0;
	int autoTimer = 0;
	TalonSRX *climber;
	double driveAngle;
	//bool isDrivingStraight = false;
	//bool wasButtonPushed = false;
	bool isRobotCentric = false;
	double autoTurnToBoilerAngle = 0.0;

	const std::string autoNameDefault = "left";
	std::string autoSelected;

	// Servo constants
	const double kServoRightOpen   =  0.5;
	const double kServoRightClosed = -0.5;
	const double kServoLeftOpen    = -0.1;
	const double kServoLeftClosed  =  0.5;


	// Tunable parameters for target detection
	const int kTrackingBrightness = 22;
	const int kNormalBrightness   = 128;
	const int kGearTargetWidth    = 20;
	const int kGearTargetHeight   = 50;
	const int kGearTargetArea = (kGearTargetWidth * kGearTargetHeight);

	// Tunable parameters for driving
	const double kSpinRateLimiter = 0.3;

	// Tunable parameters for autonomous
	const double kAutoSpeed = 0.5;



	// Tunable parameters for the PID Controllers
	const double kP = 0.01;
	const double kI = 0.00;
	const double kD = 0.01;
	const double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */
	const double kToleranceDegrees = 1.0;
	const double kToleranceStrafe = 5.0;

	enum AutoState {
		kDoNothing,
		kTurningToSpring,
		kDrivingForward,
		kPlacingGear,
		kDrivingBackward,
		kTurningToBoiler,
		kDrivingToLaunchPosition,
		kLaunching
	} autoState = kDoNothing;

	PixyTracker *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_targets[2];

	RobotDrive *drive;
	TalonSRX   *lFrontMotor;
	TalonSRX   *lBackMotor;
	TalonSRX   *rFrontMotor;
	TalonSRX   *rBackMotor;
	Joystick   *stick;
	AHRS       *ahrs; // navX MXP

	PIDController *turnController;      // PID Controller
	PIDController *strafeController;    // PID Controller

	// Output from the Turn (Angle) PID Controller
	class TurnPIDOutput : public PIDOutput {
	public:
		double correction;
		void PIDWrite(double output) {
			correction = output;
		}
	} turnPIDOutput;

	// Output from the Strafe PID Controller
	class StrafePIDOutput : public PIDOutput {
	public:
		double correction;
		void PIDWrite(double output) {
			correction = output;
		}
	} strafePIDOutput;

	// Data source for the Strafe PID Controller
	class StrafePIDSource : public PIDSource {
	public:
		StrafePIDSource() : offset(0.0) {}
		double PIDGet() {
			return offset;
		}
		void calcTargetOffset(PixyTracker::Target* targets, int count) {
			if (count < 2) {
				offset = 0.0;
			} else {
				offset = (targets[0].block.x + targets[1].block.x)/2.0 - 160.0;
			}
		}
	private:
		double offset;
	} strafePIDSource;

	float slider = 1.0;

	// Deadband filter
	float deadBand(float stickValue) {
		if(stickValue <= .15 && stickValue >= -.15) {
			return 0.0;
		} else {
			return stickValue;
		}
	}

	void autoTurningToSpring() {
		// TODO
		drive->MecanumDrive_Cartesian(0, 0, kSpinRateLimiter*turnPIDOutput.correction, ahrs->GetAngle());
		if (turnPIDOutput.correction < kToleranceDegrees) {
			autoState = kDrivingForward;
		}
	}

	void autoDrivingForward() {

		autoTimer++;

		std::cout << "auto timer:" << autoTimer << std::endl;

		if(autoTimer <= 175)
		{
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, -kAutoSpeed, -turnPIDOutput.correction, ahrs->GetAngle());
		}
		else
		{
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
			autoState = kPlacingGear;
			autoTimer = 0;
		}

		//drive->MecanumDrive_Cartesian(kAutoSpeed*strafePIDOutput.correction,
		//		kAutoSpeed,
		//		kSpinRateLimiter*turnPIDOutput.correction,
		//		ahrs->GetAngle());

		// Targets from Pixy and calculate the offset from center
		//int count = m_pixy->getBlocksForSignature(m_signature, 2, m_gear_targets);
		//strafePIDSource.calcTargetOffset(m_gear_targets, count);

		//std::cout << "turn correction = " << turnPIDOutput.correction << " strafe correction = "
		//		  << strafePIDOutput.correction << std::endl;

		//if (2 == count) {
		//	if ((m_gear_targets[0].block.width > kGearTargetWidth)   &&
		//			(m_gear_targets[0].block.height > kGearTargetHeight) &&
		//			(m_gear_targets[1].block.width > kGearTargetWidth)   &&
		//			(m_gear_targets[1].block.height > kGearTargetHeight) &&
		//			(abs(m_gear_targets[0].block.y - m_gear_targets[1].block.y)) < 10) {
		//		std::cout << "Stopped" << std::endl;
		//		autoState = kDoNothing;
		//	}
		//}
	}

	void autoPlacingGear() {
		autoTimer++;
		if (autoTimer < 125)
		{
			gearServoRight -> Set(0.5);
			gearServoLeft -> Set(-0.1);
		}
		else
		{
			autoTimer = 0;
			autoState = kDrivingBackward;
		}
		// TODO
	}

	void autoDrivingBackward() {
		autoTimer++;
		if (autoTimer < 125)
		{
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, kAutoSpeed, -turnPIDOutput.correction, 0.0);
		}
		else if(autoTimer < 100)
		{
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
			gearServoRight -> Set(-0.5);
			gearServoLeft -> Set(0.5);
		}
		else
		{
			autoTimer = 0;
			autoState = kTurningToBoiler;
		}
	}

	void autoTurningToBoiler() {
		autoTimer++;
		if (autoTimer < 100 && autoSelected == "right")
		{
			turnController->SetSetpoint(autoTurnToBoilerAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction * 0.5, ahrs->GetAngle());
		}
		else if (autoTimer < 2000)
		{
			int count = m_pixy->getBlocksForSignature(m_signature, 2, m_targets);
			int avg=0;
			for (int i=0; i<count; i++) {
				std::cout << i << "  " << m_targets[i].block.x << "  " << m_targets[i].block.y << std::endl;
				avg+= m_targets[i].block.x;
			}
			avg = avg/count;

			std::cout << "avg = " << avg << std::endl;;
			if (avg < 155) {
				//drive->MecanumDrive_Cartesian(0.0, 0.0, 0.25*diff, ahrs->GetAngle());
				autoTurnToBoilerAngle-=1.0;
				turnController->SetSetpoint(autoTurnToBoilerAngle);
				turnController->Enable();
				drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction * 0.5, ahrs->GetAngle());
			} else if (avg > 165) {
				//drive->MecanumDrive_Cartesian(0.0, 0.0, -0.25*diff, ahrs->GetAngle());
				autoTurnToBoilerAngle+=1.0;
				turnController->SetSetpoint(autoTurnToBoilerAngle);
				turnController->Enable();
				drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction * 0.5, ahrs->GetAngle());
			} else {
				drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle());
				autoTimer = 0;
				autoState = kDrivingToLaunchPosition;
			}
		}
		else
		{
			// No target found, how sad!
			autoTimer = 0;
			autoState = kDoNothing;
		}
	}

	void autoDrivingToLaunchPosition() {
		autoTimer++;

		if (autoTimer < 200)
		{
			int count = m_pixy->getBlocksForSignature(m_signature, 2, m_targets);
			std::cout << "target count: count:" << count << std::endl;

			turnController->SetSetpoint(autoTurnToBoilerAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, -kAutoSpeed, -turnPIDOutput.correction, ahrs->GetAngle());

			int avg=0;
			for (int i=0; i<count; i++) {
				std::cout << i << "  " << m_targets[i].block.width << "  " << m_targets[i].block.width << std::endl;
				avg+= m_targets[i].block.width;
			}
			avg = avg/count;

			if (avg > 30) {
				autoTimer = 0;
				autoState = kLaunching;
			}
		} else {
			// Lost target?
			autoTimer = 0;
			autoState = kDoNothing;
		}
	}

	void autoLaunching() {
		++autoTimer;

		if (autoTimer < 1000) {
			shooter -> SetTalonControlMode(CANTalon::kSpeedMode);
			shooter -> Set(4100.0);

			if (autoTimer == 50) {
				autoTimer = 0;
				feederMotor ->Set(0.3);
			}
		} else {
			feederMotor -> Set(0.0);
			shooter -> Set(0.0);
			autoTimer = 0;
			autoState = kDoNothing;
		}
	}

	void autoDoNothing() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle()); //stop
	}

	void RobotInit()
	{
		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &chooser);

		// gearServo is left, gearServo2 is right
		gearServoRight = new Servo(5); //right servo on practice bot
		gearServoLeft = new Servo(3); //left is 3 on practice bot
		climber = new TalonSRX(7);
		stick        = new Joystick(0);
		lFrontMotor  = new TalonSRX(0); // 9
		lBackMotor	 = new TalonSRX(8); // 1
		rFrontMotor  = new TalonSRX(1); // 8
		rBackMotor	 = new TalonSRX(9); // 0

		feederMotor = new TalonSRX(6);
		rFrontMotor->SetInverted(true);
		rBackMotor->SetInverted(true);
		shooter = new CANTalon(0);
		shooter -> SetTalonControlMode(CANTalon::kSpeedMode);
		shooter -> Set(4000.0);
		shooter -> SetSensorDirection(true);
		shooter -> ConfigNominalOutputVoltage(+0., -0.

		);

		shooter
		-> ConfigPeakOutputVoltage(+12., 0.);
		shooter -> SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		shooter->SetF(0.024);	// 775 Pro
		shooter->SetP(0.035);	// 775 Pro
		shooter->SetI(0.00);	// 775 Pro
		shooter->SetD(0.1);
		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}

		drive = new RobotDrive(lFrontMotor, lBackMotor, rFrontMotor, rBackMotor);
		drive->SetSafetyEnabled(false);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, &turnPIDOutput);
		turnController->SetInputRange(-180.0, 180.0);
		turnController->SetOutputRange(-1.0, 1.0);
		turnController->SetAbsoluteTolerance(kToleranceDegrees);
		turnController->SetContinuous(true);

		strafeController = new PIDController(kP, kI, kD, kF, &strafePIDSource, &strafePIDOutput);
		strafeController->SetInputRange(-100.0, 100.0);
		strafeController->SetOutputRange(-1.0, 1.0);
		strafeController->SetAbsoluteTolerance(kToleranceStrafe);
		strafeController->SetContinuous(true);

		// Create the Pixy instance and start streaming the frames
		m_pixy = new PixyTracker();
		m_pixy->startVideo();
	}

	void AutonomousInit() {
		m_pixy->setTiltandBrightness(kTrackingBrightness, 0);
        shooter -> Set(0.0);
		autoState = kDrivingForward; // Initial state
		ahrs->ZeroYaw();             // Initialize to zero

		autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == "right") {
			autoTurnToBoilerAngle = 90.0;
		} else {
			autoTurnToBoilerAngle = -90.0;
		}

		//turnController->SetSetpoint(0.0);
		//turnController->Enable();
		//strafeController->SetSetpoint(0.0);
		//strafeController->Enable();
	}


	void DisabledInit() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
		shooter -> Set(0.0);
		feederMotor ->Set(0.0);
	}

	void AutonomousPeriodic() {
		//std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
		//	lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		//std::cout << "Angle: " << ahrs->GetAngle() << "  Rate: " << ahrs->GetRate() << std::endl;

		switch (autoState) {
		case kTurningToSpring:
			autoTurningToSpring();
			break;
		case kDrivingForward:
			autoDrivingForward();
			break;
		case kPlacingGear:
			autoPlacingGear();
			break;
		case kDrivingBackward:
			autoDrivingBackward();
			break;
		case kTurningToBoiler:
			autoTurningToBoiler();
			break;
		case kDrivingToLaunchPosition:
			autoDrivingToLaunchPosition();
			break;
		case kLaunching:
			autoLaunching();
			break;
		default:
		case kDoNothing:
			autoDoNothing();
			break;
		}
	}

	void TeleopInit() {
		m_pixy->setTiltandBrightness(kNormalBrightness, 1);
	}

	void TeleopPeriodic() { // 0: leftX, 1: leftY, 2: left trigger, 3: right trigger, 4: rightX, 5: rightY

		//SmartDashboard::PutNumber("Gyro Angle", ahrs->GetAngle());


		// Competition Robot Servos
		/*if (stick -> GetRawButton(6)){
			gearServoRight -> Set(.5);

			gearServoLeft -> Set(.3);

		} else {
			gearServoRight -> Set(.9);
			gearServoLeft -> Set(0);
		}*/

		// Practice Robot
		if (stick -> GetRawButton(6)) { //gearservo is right, gearservo2 is left
			gearServoRight -> Set(kServoRightOpen);
			gearServoLeft -> Set(kServoLeftOpen);

		} else {
			gearServoRight -> Set(kServoRightClosed);
			gearServoLeft -> Set(kServoLeftClosed);
		}

		if (stick -> GetRawAxis(3)) {
			++timer;
			speed = shooter -> GetSpeed();
			shooter -> SetTalonControlMode(CANTalon::kSpeedMode);
			shooter -> Set(4100.0);

			std::cout << "Speed = " << speed << std::endl;
			if (timer == 50) {
				timer = 0;
				feederMotor ->Set(.3);
			}
		} else {
			timer = 0;
			feederMotor -> Set(0.0);
			shooter -> Set(0.0);
		}

		if (stick->GetRawAxis(2)){
			climber -> Set(1.0);
		} else{
			climber -> Set(0.0);
		}

		// Check for drive-mode toggle
		/*
		if (stick->GetRawButton(1)) {
			if (!wasButtonPushed) {
				isRobotCentric = !isRobotCentric;

				if (isRobotCentric) {
					driveAngle = ahrs->GetAngle();
					double rotations = floor(abs(driveAngle)/360.0);
					if (driveAngle < 0.0) {
						driveAngle+= rotations*360.0;
					} else {
						driveAngle-= rotations*360.0;
					}
					if (driveAngle > 180.0) {
						driveAngle-=360.0;
					} else if (driveAngle < -180.0) {
						driveAngle+=360.0;
					}

				std::cout << "set drive angle\n";
				}
			}

			wasButtonPushed = true;
		} else {
			wasButtonPushed = false;
		}*/

		if (stick->GetRawButton(1) || stick->GetRawButton(2)) {
			if (!isRobotCentric) {

				isRobotCentric = true;

				driveAngle = ahrs->GetAngle();
				double rotations = floor(abs(driveAngle)/360.0);
				if (driveAngle < 0.0) {
					driveAngle+= rotations*360.0;
				} else {
					driveAngle-= rotations*360.0;
				}
				if (driveAngle > 180.0) {
					driveAngle-=360.0;
				} else if (driveAngle < -180.0) {
					driveAngle+=360.0;
				}
			}
		} else {
			isRobotCentric = false;
		}

		if (isRobotCentric) {
			std::cout << "Robot Centric: " << ahrs->GetAngle() << " " << driveAngle << " " << turnPIDOutput.correction << std::endl;
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();

			if (stick->GetRawButton(2)) {
				drive->MecanumDrive_Cartesian(
								deadBand(0.0),
								deadBand(stick->GetRawAxis(1)),
								-turnPIDOutput.correction,
								ahrs->GetAngle());
			} else {
			drive->MecanumDrive_Cartesian(
					deadBand(stick->GetRawAxis(0)),
					deadBand(stick->GetRawAxis(1)),
					-turnPIDOutput.correction,
					ahrs->GetAngle());
			}
		} else {
			turnController->Disable();

		/* drive->MecanumDrive_Cartesian(
				slider * deadBand(stick->GetRawAxis(0)),
				slider * deadBand(stick->GetRawAxis(1)),
				kSpinRateLimiter * (-1)*deadBand(stick->GetRawAxis(2)),
				ahrs->GetAngle()); */
		drive->MecanumDrive_Cartesian(
				deadBand(stick->GetRawAxis(0)),
				deadBand(stick->GetRawAxis(1)),
				kSpinRateLimiter * (-1)*deadBand(stick->GetRawAxis(4)),
				ahrs->GetAngle());
		}
		// DEBUG
		//std::cout << std::setprecision(3) << std::fixed;
		//std::cout << "Axis 0: " << stick->GetRawAxis(0) << "  Axis 1: " << stick->GetRawAxis(1) << "  Axis 4: " << stick->GetRawAxis(4) << std::endl;

		//std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
		//		lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		//std::cout << "Angle: " << ahrs->GetAngle() << "  Rate: " << ahrs->GetRate() << std::endl;
		//std::cout << "Angle: " << ahrs->GetAngle()
		//		  << "  Yaw: " << ahrs->GetYaw()
		//		  << "  Pitch: " << ahrs->GetPitch()
		//		  << "  Roll: " << ahrs->GetRoll() << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
