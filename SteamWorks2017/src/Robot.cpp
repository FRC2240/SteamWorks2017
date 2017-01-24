#include <iostream>
#include <iomanip>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"

class Robot: public IterativeRobot {
private:
	// Tunable parameters for target detection
	const int kTrackingBrightness = 22;
	const int kNormalBrightness   = 128;
	const int kGearTargetWidth    = 20;
	const int kGearTargetHeight   = 50;
	const int kGearTargetArea = (kGearTargetWidth * kGearTargetHeight);

	// Tunable parameters for driving
	const double kSpinRateLimiter = 0.2;

	// Tunable parameters for the PID Controllers
	const double kP = 0.03;
	const double kI = 0.00;
	const double kD = 0.00;
	const double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */
	const double kToleranceDegrees = 0.2;
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

	// Configure for tracking
	void startTracking() {
		// Reset brightness
		m_pixy->setForTracking(kTrackingBrightness);
	}

	// Unconfigure tracking
	void stopTracking() {
		// Reset to a fixed pan/tilt position

		// Reset brightness
		m_pixy->setForDriver(kNormalBrightness);
		m_pixy->clearTargets();
	}

	void autoTurningToSpring() {
		// TODO
		drive->MecanumDrive_Cartesian(0, 0, kSpinRateLimiter*turnPIDOutput.correction, ahrs->GetAngle());
		if (turnPIDOutput.correction < kToleranceDegrees) {
			autoState = kDrivingForward;
		}
	}

	void autoDrivingForward() {
		drive->MecanumDrive_Cartesian(0.2*strafePIDOutput.correction,
				0.2,
				kSpinRateLimiter*turnPIDOutput.correction,
				ahrs->GetAngle());

		// Targets from Pixy and calculate the offset from center
		int count = m_pixy->getBlocksForSignature(m_signature, 2, m_gear_targets);
		strafePIDSource.calcTargetOffset(m_gear_targets, count);

		std::cout << "turn correction = " << turnPIDOutput.correction << " strafe correction = "
				  << strafePIDOutput.correction << std::endl;

		if (2 == count) {
			if ((m_gear_targets[0].block.width > kGearTargetWidth)   &&
				(m_gear_targets[0].block.height > kGearTargetHeight) &&
				(m_gear_targets[1].block.width > kGearTargetWidth)   &&
				(m_gear_targets[1].block.height > kGearTargetHeight) &&
				(abs(m_gear_targets[0].block.y - m_gear_targets[1].block.y)) < 10) {
				std::cout << "Stopped" << std::endl;
				autoState = kDoNothing;
			}
		}
	}

	void autoPlacingGear() {
		// TODO
	}

	void autoDrivingBackward() {
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
		stick        = new Joystick(0);
		lFrontMotor  = new TalonSRX(0);
		lBackMotor	 = new TalonSRX(8);
		rFrontMotor  = new TalonSRX(1);
		rBackMotor	 = new TalonSRX(9);

		rFrontMotor->SetInverted(true);
		rBackMotor->SetInverted(true);

		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}

		drive = new RobotDrive(lFrontMotor, lBackMotor, rFrontMotor, rBackMotor);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, &turnPIDOutput);
		turnController->SetInputRange(-180.0f,  180.0f);
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
		startTracking();

		autoState = kDrivingForward; // Initial state
		ahrs->ZeroYaw();             // Initialize to zero

		turnController->SetSetpoint(0.0);
		turnController->Enable();
		strafeController->SetSetpoint(0.0);
		strafeController->Enable();
	}

	void DisabledInit() {
		stopTracking();
		turnController->Disable();
		strafeController->Disable();
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
	}

	void TeleopPeriodic() { // 0: leftX, 1: leftY, 2: left trigger, 3: right trigger, 4: rightX, 5: rightY
		slider = 1.0; //(stick->GetRawAxis(4)+1)/2;

		drive->MecanumDrive_Cartesian(
				slider * deadBand(stick->GetRawAxis(0)),
				slider * deadBand(stick->GetRawAxis(1)),
				kSpinRateLimiter * (-1)*deadBand(stick->GetRawAxis(2)),
				ahrs->GetAngle());

		// DEBUG
		std::cout << std::setprecision(3) << std::fixed;
		//std::cout << "Axis 0: " << stick->GetRawAxis(0) << "  Axis 1: " << stick->GetRawAxis(1) << "  Axis 4: " << stick->GetRawAxis(4) << std::endl;

		std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
				lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		std::cout << "Angle: " << ahrs->GetAngle() << "  Rate: " << ahrs->GetRate() << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
