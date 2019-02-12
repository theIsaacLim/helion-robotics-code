/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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
  public static VictorSP rightGrabber = new VictorSP(5);
  public static Talon leftGrabber = new Talon(4);

  // This part is wheely cool!
  public static Talon frontLeftWheel = new Talon(0);
  public static Talon frontRightWheel = new Talon(1);
  public static Talon backLeftWheel = new Talon(2);
  public static Talon backRightWheel = new Talon(3);
  public static SpeedControllerGroup left = new SpeedControllerGroup(frontLeftWheel, backLeftWheel);
  public static SpeedControllerGroup right = new SpeedControllerGroup(frontRightWheel, backRightWheel);
  //public static RobotDrive myDrive = new RobotDrive(frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel);
  public static DifferentialDrive secondDrive = new DifferentialDrive(left, right);
  // Controller
  public static Joystick leftJoy = new Joystick(0);
  // public static Joystick rightJoy = new Joystick(0);
}
