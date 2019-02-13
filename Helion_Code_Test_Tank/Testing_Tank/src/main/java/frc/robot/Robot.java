/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  WPI_TalonSRX frontLeftMotor;
  WPI_TalonSRX frontRightMotor;

  WPI_TalonSRX backLeftMotor;
  WPI_TalonSRX backRightMotor;

  WPI_TalonSRX majorElevator;

  private DifferentialDrive drive;
  private Joystick stick;

  private double sensitivity;

  private DigitalInput majElevatorTopSwitch;
  private DigitalInput majElevatorDownSwitch;

  private boolean topFirstHit;

  private UsbCamera camera;

  @Override
  public void robotInit() {
    frontLeftMotor = new WPI_TalonSRX(RobotMap.flChannel);
    frontRightMotor = new WPI_TalonSRX(RobotMap.frChannel);

    backLeftMotor = new WPI_TalonSRX(RobotMap.blChannel);
    backRightMotor = new WPI_TalonSRX(RobotMap.brChannel);

    majorElevator = new WPI_TalonSRX(RobotMap.majElevatorChannel);

    drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    stick = new Joystick(RobotMap.joyChannel);

    sensitivity = 0.5;

    majElevatorTopSwitch = new DigitalInput(RobotMap.lSMajEleUp);
    majElevatorDownSwitch = new DigitalInput(RobotMap.lSMajEleDown);

    topFirstHit = true;

    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);
    camera.setFPS(15);
  }

  @Override
  public void teleopInit() {
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    backLeftMotor.configFactoryDefault();
    backRightMotor.configFactoryDefault();

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backLeftMotor.setInverted(InvertType.FollowMaster);
    backRightMotor.setInverted(InvertType.FollowMaster);

    drive.setRightSideInverted(false);
  }

  @Override
  public void teleopPeriodic() {
    System.out.println("Switch: " + majElevatorDownSwitch.get());
    double forward = -stick.getY();
    double turn = stick.getX() * sensitivity;
    double sliderSensitivity = (stick.getRawAxis(3) + 1) * 0.5;
    // System.out.println("JoyY:" + forward + " turn:" + turn );
    drive.arcadeDrive(forward * sliderSensitivity, turn * sliderSensitivity);

    if (stick.getRawButton(RobotMap.joyMajorElevatorUp)) {
      topFirstHit = true;
      if (!majElevatorTopSwitch.get()) {
        if (topFirstHit) {
          majorElevator.set(1);
        } else {
          majorElevator.set(0.5);
        }
      } else {
        if (topFirstHit) {
          topFirstHit = false;
        }
        majorElevator.set(0);
      }
    } else if (stick.getRawButton(RobotMap.joyMajorElevatorDown)) {
      if(majElevatorDownSwitch.get()){
        majorElevator.set(-1);
      }else{
        majorElevator.set(0);
      }
    } else {
      majorElevator.set(0);
    }
    // System.out.println(stick.getPOV());
  }

  public void reachLevelOne() {

  }
}
