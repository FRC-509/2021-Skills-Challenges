#pragma once

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxDriver.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <adi/ADIS16470_IMU.h>

extern frc::ADIS16470_IMU imu;

//Falcon Motor Controller Declaration
extern TalonSRX leftFrontFalcon;
extern TalonSRX leftBackFalcon;
extern TalonSRX rightFrontFalcon;
extern TalonSRX rightBackFalcon;
//Shooter
// Right should be negative and left should be positive
extern TalonSRX l_shooter;
extern TalonSRX r_shooter;
//MCGintake
extern TalonSRX MCGintake;
//Skywalker
extern TalonSRX skywalker;

//CURRENT LIMITING
extern WPI_TalonFX * shooterEncoder;
//ConfigPeakCurrentLimit();

//SparkMax Motor Declaration
//Color Wheel Motor
extern rev::CANSparkMax colorWheelMotor;
//Lift moves powercells from belt to turret
extern rev::CANSparkMax lift1;
extern rev::CANSparkMax lift2;
//Elevator
extern rev::CANSparkMax elevator; 
extern rev::CANEncoder elevatorPoint;
//Rotation of Shooter
extern rev::CANSparkMax turret;
extern rev::CANEncoder turretEncode;
//Hood controls angle of Shooter
extern rev::CANSparkMax hood;
extern rev::CANEncoder hoodEncoder;
//Conveyor Belt and Lift
extern rev::CANSparkMax belt;

extern frc::Compressor compressor;
//One of these is up and the other is down ?
extern frc::Solenoid intakeSolOpen;
extern frc::Solenoid intakeSolClose;
extern frc::Solenoid brakeSolOff;
extern frc::Solenoid colorSolUp;
extern frc::Solenoid brakeSolOn;
extern frc::Solenoid colorSolDown;
//Digital Sensors
extern frc::DigitalInput sensorIntake;
extern frc::DigitalInput sensorExit;