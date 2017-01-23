#include <iostream>
#include <iomanip>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"

class Robot: public IterativeRobot {
private:
	enum TrackingMode {
		kNoTracking,
		kGearTracking,
		kBoilerTracking
	} trackingMode = kNoTracking;

	enum AutoState {
		kDoNothing,
		kTurning,
		kDrivingForward,
		kDrivingBackward
	} autoState = kDoNothing;

	const int kTrackingBrightness = 20;
	const int kNormalBrightness = 128;

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

	const double kP = 0.03;
	const double kI = 0.00;
	const double kD = 0.00;
	const double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */
	const double kToleranceDegrees = 0.2;
	const double kToleranceStrafe = 5.0;

	//
	class TurnPIDOutput : public PIDOutput {
	public:
		double correction;
		void PIDWrite(double output) {
			correction = output;
		}
	} turnPIDOutput;

	//
	class StrafePIDOutput : public PIDOutput {
	public:
		double correction;
		void PIDWrite(double output) {
			correction = output;
		}
	} strafePIDOutput;

	//
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

	}

	// Unconfigure tracking
	void stopTracking() {
		// Reset to a fixed pan/tilt position
		// Reset brightness

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
		autoState = kDrivingForward;

		// Initialize to zero
		ahrs->ZeroYaw();
		m_pixy->setForTracking(kTrackingBrightness);

		turnController->SetSetpoint(0.0);
		turnController->Enable();
		strafeController->SetSetpoint(0.0);
		strafeController->Enable();
	}

	void AutonomousDisabled() {
		std::cout << "Autonomous Disabled\n";
		m_pixy->setForDriver(kNormalBrightness);
		turnController->Disable();
		strafeController->Disable();
	}

	void AutonomousPeriodic() {
		/*if (kTurning == autoState) {
			drive->MecanumDrive_Cartesian(0, 0, turnPIDOutput.turnOutput, ahrs->GetAngle());
			if (turnPIDOutput.turnOutput < kToleranceDegrees) {
				autoState = kDrivingForward;
				strafeController->SetSetpoint(0.0);
				strafeController->Enable();
			}
		}*/

		std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
			lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		std::cout << "Angle: " << ahrs->GetAngle() << "  Rate: " << ahrs->GetRate() << std::endl;

		if (kDrivingForward == autoState) {
			int count = m_pixy->getBlocksForSignature(m_signature, 2, m_gear_targets);
			strafePIDSource.calcTargetOffset(m_gear_targets, count);

			std::cout << "turn correction   = " << turnPIDOutput.correction << std::endl;
			std::cout << "strafe correction = " << strafePIDOutput.correction << std::endl;

			drive->MecanumDrive_Cartesian(strafePIDOutput.correction,
					                      0.2,
									      turnPIDOutput.correction,
										  ahrs->GetAngle());
			if (2 == count) {
				if (m_gear_targets[0].block.width > 40) {
					//stop
					std::cout << "Stopped" << std::endl;
					drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle());
				}
			}
		}


	}

	void TeleopInit() {
		trackingMode = kNoTracking;
	}

	void TeleopPeriodic() { // 0: leftX, 1: leftY, 2: left trigger, 3: right trigger, 4: rightX, 5: rightY
		slider = 1.0; //(stick->GetRawAxis(4)+1)/2;

		drive->MecanumDrive_Cartesian(
				0.2, 0.0, 0.0,
				//slider * deadBand(stick->GetRawAxis(0)),
				//slider * deadBand(stick->GetRawAxis(1)),
				//0.2 * (-1)*deadBand(stick->GetRawAxis(2)),
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
