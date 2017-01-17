#include <iostream>
#include <iomanip>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"

class Robot: public IterativeRobot {
private:
	enum TrackingMode {
		kNoTracking = 0,
		kGearTracking,
		kBoilerTracking
	} trackingMode = kNoTracking;

	const int kTrackingBrightness = 20;
	const int kNormalBrightness = 80;

	PixyTracker *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_target;

	RobotDrive *drive;
	TalonSRX   *lFrontMotor;
	TalonSRX   *lBackMotor;
	TalonSRX   *rFrontMotor;
	TalonSRX   *rBackMotor;
	Joystick   *stick;
	AHRS       *ahrs; // navX MXP

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

	// Process joystick input from driver
	void doDriverControl() {
		slider = (stick->GetRawAxis(4)+1)/2;

		drive->MecanumDrive_Cartesian(
				slider * deadBand(stick->GetRawAxis(0)),
				slider * deadBand(stick->GetRawAxis(1)),
				slider * (-1)*deadBand(stick->GetRawAxis(2)),
				ahrs->GetAngle());
	}

	// Drive while tracking the gear targets, place the gear, then back up
	void doGearTracking() {

	}

	//  Drive while tracking the boiler targets to the correct distance, launch fuel
	void doBoilerTracking() {

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

		// Create the Pixy instance and start streaming the frames
		m_pixy = new PixyTracker();
		m_pixy->startVideo();
	}

	void AutonomousInit() {
		// Initialize to zero
		ahrs->ZeroYaw();
	}

	void AutonomousPeriodic() {
		switch (trackingMode) {
		case kNoTracking:
			break;
		case kGearTracking:
			break;
		case kBoilerTracking:
			break;
		}
	}

	void TeleopInit() {
		trackingMode = kNoTracking;
	}

	void TeleopPeriodic() { // 0: leftX, 1: leftY, 2: left trigger, 3: right trigger, 4: rightX, 5: rightY
		switch (trackingMode) {
		case kNoTracking:
			doDriverControl();
			break;
		case kGearTracking:
			doGearTracking();
			break;
		case kBoilerTracking:
			doBoilerTracking();
			break;
		}

		// DEBUG
		std::cout << std::setprecision(3) << std::fixed;
		std::cout << "Axis 0: " << stick->GetRawAxis(0) << "  Axis 1: " << stick->GetRawAxis(1) << "  Axis 4: " << stick->GetRawAxis(4) << std::endl;

		std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
				lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		std::cout << "Angle: " << ahrs->GetAngle() << "  Rate: " << ahrs->GetRate() << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
