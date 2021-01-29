//Falcon Motor Controller Declaration
extern TalonSRX leftFrontFalcon = {0};
extern TalonSRX leftBackFalcon = {1};
extern TalonSRX rightFrontFalcon = {2};
extern TalonSRX rightBackFalcon = {3};
//Shooter
// Right should be negative and left should be positive
externTalonSRX l_shooter = {4};
extern TalonSRX r_shooter = {5};
//MCGintake
extern TalonSRX MCGintake;
//Skywalker
extern TalonSRX skywalker = {7};

//CURRENT LIMITING
extern WPI_TalonFX * shooterEncoder = new WPI_TalonFX{4};
//ConfigPeakCurrentLimit();

//SparkMax Motor Declaration
//Color Wheel Motor
extern rev::CANSparkMax colorWheelMotor;
//Lift moves powercells from belt to turret
extern rev::CANSparkMax lift1 ;
extern rev::CANSparkMax lift2 ;
//Elevator
extern rev::CANSparkMax elevator; 
extern rev::CANEncoder elevatorPoint = elevator.GetEncoder();
//Rotation of Shooter
extern rev::CANSparkMax turret;
extern rev::CANEncoder turretEncode = turret.GetEncoder();
//Hood controls angle of Shooter
extern rev::CANSparkMax hood;
extern rev::CANEncoder hoodEncoder = hood.GetEncoder();
//Conveyor Belt and Lift
extern rev::CANSparkMax belt;