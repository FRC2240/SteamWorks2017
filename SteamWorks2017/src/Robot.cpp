#include <iostream>
#include <iomanip>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"
#include <CANTalon.h>

class Robot: public IterativeRobot {
private:


	//Feeder and shooter motor
	Servo *gearServo;
	Servo *gearServo2;
	TalonSRX *feederMotor;
	CANTalon *shooter;
	double speed;
	int timer = 0;
	int autoTimer = 0;
	TalonSRX *climber;
	double driveAngle;
	bool isDrivingStraight = false;
	bool wasButtonPushed = false;
	bool isRobotCentric = false;


	// Tunable parameters for target detection
	const int kTrackingBrightness = 22;
	const int kNormalBrightness   = 128;
	const int kGearTargetWidth    = 20;
	const int kGearTargetHeight   = 50;
	const int kGearTargetArea = (kGearTargetWidth * kGearTargetHeight);

	// Tunable parameters for driving
	const double kSpinRateLimiter = 0.2;
	const double kAutoSpeed = 0.2;

	// Tunable parameters for the PID Controllers
	const double kP = 0.003;
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
	PixyTracker::Target m_gear_targets[2];

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

		if(autoTimer <= 500)
		{
			drive->MecanumDrive_Cartesian(0.0, 0.5, 0.0, ahrs->GetAngle());
		}
		else
		{
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
		if(autoTimer < 150)
		{
			gearServo -> Set(0.3);
			gearServo2 -> Set(0.1);
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
		if(autoTimer < 250)
		{
			drive->MecanumDrive_Cartesian(0.0, -0.5, 0.0, ahrs->GetAngle());
		}
		else if(autoTimer < 275)
		{
			gearServo -> Set(-0.2);
			gearServo2 -> Set(0.5);
		}
		else
		{
			autoTimer = 0;
			autoState = kDoNothing;//kTurningToBoiler;
		}
		// TODO
	}

	void autoTurningToBoiler() {
		// TODO
	}

	void autoDrivingToLaunchPosition() {
		// TODO
	}

	void autoLaunching() {
		// TODO
	}

	void autoDoNothing() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle()); //stop

	}

	void RobotInit()
	{
		// gearServo is left, gearServo2 is right
		gearServo = new Servo(5);
		gearServo2 = new Servo(3);
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

		turnController = new PIDController(kP, kI, kD, kF, ahrs, &turnPIDOutput);
		turnController->SetInputRange(-180.0, 180.0);
		turnController->SetOutputRange(-0.25, 0.25);
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
		m_pixy->setTiltandBrightness(kTrackingBrightness, PixyTracker::kLevelTilt);

		autoState = kDrivingForward; //kDrivingForward; // Initial state
		ahrs->ZeroYaw();             // Initialize to zero

		//turnController->SetSetpoint(0.0);
		//turnController->Enable();
		//strafeController->SetSetpoint(0.0);
		//strafeController->Enable();
	}


	void DisabledInit() {
		m_pixy->setTiltandBrightness(kNormalBrightness, PixyTracker::kDefaultTilt);
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
		// TODO
	}

	void TeleopPeriodic() { // 0: leftX, 1: leftY, 2: left trigger, 3: right trigger, 4: rightX, 5: rightY

		// Competition Robot Servos
		/*if (stick -> GetRawButton(6)){
			gearServo -> Set(.5);

			gearServo2 -> Set(.3);

		} else {
			gearServo -> Set(.9);
			gearServo2 -> Set(0);
		}*/

		// Practice Robot
		if (stick -> GetRawButton(6)) {
			gearServo -> Set(0.3);
			gearServo2 -> Set(0.1);
		} else {
			gearServo -> Set(-0.2);
			gearServo2 -> Set(0.5);
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

					std::cout << "set drive angle\n";
				}
			}
			wasButtonPushed = true;
		} else {
			wasButtonPushed = false;
		}
		if (isRobotCentric) {
			std::cout << "Robot Centric: " << ahrs->GetAngle() << " " << driveAngle << " " << turnPIDOutput.correction << std::endl;
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();

			drive->MecanumDrive_Cartesian(
					deadBand(stick->GetRawAxis(4)),
					deadBand(stick->GetRawAxis(1)),
					-turnPIDOutput.correction,
					0.0);
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
