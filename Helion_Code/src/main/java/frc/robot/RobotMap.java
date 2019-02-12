/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static int frontLeftChannel = 0;
  public static int backLeftChannel = 1;
  public static int frontRightChannel = 2;
  public static int backRightChannel = 3;

  // public static VictorSP rightGrabber = new VictorSP(5);
  // public static Talon leftGrabber = new Talon(4);

  // This part is wheely cool!
  public static TalonSRX lf;

  rf = new TalonSRX(RobotMap.frontRightChannel);
  rb = new TalonSRX(RobotMap.backRightChannel);
  lf = new TalonSRX(RobotMap.frontLeftChannel);
  lb = new TalonSRX(RobotMap.backLeftChannel);

  public static SpeedControllerGroup left = new SpeedControllerGroup(frontLeftWheel, backLeftWheel);
  public static SpeedControllerGroup right = new SpeedControllerGroup(frontRightWheel, backRightWheel);
  // public static RobotDrive myDrive = new RobotDrive(frontLeftWheel,
  // backLeftWheel, frontRightWheel, backRightWheel);
  public static DifferentialDrive secondDrive = new DifferentialDrive(left, right);
  // Controller
  public static Joystick leftJoy = new Joystick(0);
  // public static Joystick rightJoy = new Joystick(0);
}
