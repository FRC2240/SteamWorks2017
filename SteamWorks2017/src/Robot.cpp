#include "WPILib.h"
#include <iomanip>

class Robot: public IterativeRobot
{
private:
	RobotDrive *drive;
	TalonSRX *lFrontMotor;
	TalonSRX *lBackMotor;
	TalonSRX *rFrontMotor;
	TalonSRX *rBackMotor;
	Joystick *stick;
	ADXRS450_Gyro *gyro;

	float deadBand(float stickValue)
	{
		if(stickValue <= .2 && stickValue >= -.2)
		{
			return 0.0;
		}
		else
		{
			return stickValue;
		}
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

		gyro = new ADXRS450_Gyro();
		gyro->Calibrate();

		drive = new RobotDrive(lFrontMotor, lBackMotor, rFrontMotor, rBackMotor);
	}
	void AutonomousInit()
	{

	}
	void AutonomousPeriodic()
	{

	}
	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{ // 0: leftX, 1: leftY, 2: left trigger, 3: right trigger, 4: rightX, 5: rightY
		//
		//drive->HolonomicDrive(-deadBand(stick->GetRawAxis(4)), -deadBand(stick->GetRawAxis(5)), -deadBand(stick->GetRawAxis(1)));
		//drive->HolonomicDrive(magnitude, direction, speed);
		drive->MecanumDrive_Cartesian(stick->GetRawAxis(0), stick->GetRawAxis(1), deadBand(stick->GetRawAxis(4)), gyro->GetAngle());

		std::cout << std::setprecision(3) << std::fixed;
		std::cout << "Axis 0: " << stick->GetRawAxis(0) << "  Axis 1: " << stick->GetRawAxis(1) << "  Axis 4: " << stick->GetRawAxis(4) << std::endl;

		std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
				                    lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		std::cout << "Angle: " << gyro->GetAngle() << "  Rate: " << gyro->GetRate() << std::endl;

	}
};

START_ROBOT_CLASS(Robot);
