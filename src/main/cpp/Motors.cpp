include "Motors.h"

//Falcon Motor Controller Declaration
TalonSRX leftFrontFalcon = {0};
TalonSRX leftBackFalcon = {1};
TalonSRX rightFrontFalcon = {2};
TalonSRX rightBackFalcon = {3};
//Shooter
// Right should be negative and left should be positive
TalonSRX l_shooter = {4};
TalonSRX r_shooter = {5};
//MCGintake
TalonSRX MCGintake = {6};
//Skywalker
TalonSRX skywalker = {7};

//CURRENT LIMITING
WPI_TalonFX * shooterEncoder = new WPI_TalonFX{4};
//ConfigPeakCurrentLimit();

//SparkMax Motor Declaration
//Color Wheel Motor
rev::CANSparkMax colorWheelMotor { 8 , rev::CANSparkMax::MotorType::kBrushed};
//Lift moves powercells from belt to turret
rev::CANSparkMax lift1 { 9 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift2 { 10 , rev::CANSparkMax::MotorType::kBrushless};
//Elevator
rev::CANSparkMax elevator { 11 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder elevatorPoint = elevator.GetEncoder();
//Rotation of Shooter
rev::CANSparkMax turret { 12 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder turretEncoder = turret.GetEncoder();
//Hood controls angle of Shooter
rev::CANSparkMax hood { 13 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder hoodEncoder = hood.GetEncoder();
//Conveyor Belt and Lift
rev::CANSparkMax belt { 14 , rev::CANSparkMax::MotorType::kBrushless};