package org.usfirst.frc.team948.robot;
import edu.wpi.first.wpilibj.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
	
	public static Victor backLeftMotor = new Victor(3);
	public static Victor frontLeftMotor = new Victor(2);
	public static Victor backRightMotor = new Victor(1);
	public static Victor fromRightMotor = new Victor(0);
	//may be wrong thing that AHRS is constructed with
	public static AHRS robotAHRS = new AHRS(SPI.Port.kMXP);
	public static AHRSGyro robotGyro = new AHRSGyro();
	//To check values
	public static Encoder rightEncoder = new Encoder(0,1,false,EncodingType.k4X);
	//To check values
	public static Encoder leftEncoder = new Encoder(2,3,false,EncodingType.k4X);
}
