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

public class Robot extends TimedRobot {

  // Major speed controllers and their respective motors.

  WPI_TalonSRX frontLeftMotor;
  WPI_TalonSRX frontRightMotor;

  WPI_TalonSRX backLeftMotor;
  WPI_TalonSRX backRightMotor;

  WPI_TalonSRX majorElevator;
  WPI_TalonSRX minorElevator;

  WPI_TalonSRX mainGrabber;
  WPI_TalonSRX slaveGrabber;

  // The drive the robot uses
  private DifferentialDrive drive;
  // Joystick
  private Joystick stick;

  // Sensitivity for movement, constant.
  private double sensitivity;

  // Limit switches for the major elevator.
  private DigitalInput majElevatorTopSwitch;
  private DigitalInput majElevatorDownSwitch;

  // Limit switches for the minor (grabber) elevator.
  private DigitalInput minElevatorTopSwitch;
  private DigitalInput minElevatorDownSwitch;

  // Strings for modes of each elevator.
  private String majorElevatorMode;
  private String minorElevatorMode;

  // Boolean to control whether its the first hit of the major elevator or the
  // repeated
  // successions.
  private boolean majTopFirstHit;

  // Boolean to control the first hit of grabber elevator.
  private boolean minTopFirstHit;
  // Camera
  private UsbCamera camera;

  @Override
  public void robotInit() {
    frontLeftMotor = new WPI_TalonSRX(RobotMap.flChannel);
    frontRightMotor = new WPI_TalonSRX(RobotMap.frChannel);

    backLeftMotor = new WPI_TalonSRX(RobotMap.blChannel);
    backRightMotor = new WPI_TalonSRX(RobotMap.brChannel);

    majorElevator = new WPI_TalonSRX(RobotMap.majElevatorChannel);
    minorElevator = new WPI_TalonSRX(RobotMap.minElevatorChannel);

    mainGrabber = new WPI_TalonSRX(RobotMap.mainGrabber);
    slaveGrabber = new WPI_TalonSRX(RobotMap.slaveGrabber);

    drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    stick = new Joystick(RobotMap.joyChannel);

    sensitivity = 0.5;

    majElevatorTopSwitch = new DigitalInput(RobotMap.lSMajEleUp);
    majElevatorDownSwitch = new DigitalInput(RobotMap.lSMajEleDown);

    minElevatorTopSwitch = new DigitalInput(RobotMap.lSMinEleUp);
    minElevatorDownSwitch = new DigitalInput(RobotMap.lSMinEleDown);

    majTopFirstHit = true;
    minTopFirstHit = true;

    majorElevatorMode = "idle";
    minorElevatorMode = "idle";

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
    majorElevator.configFactoryDefault();
    minorElevator.configFactoryDefault();
    mainGrabber.configFactoryDefault();
    slaveGrabber.configFactoryDefault();

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    slaveGrabber.follow(mainGrabber);

    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backLeftMotor.setInverted(InvertType.FollowMaster);
    backRightMotor.setInverted(InvertType.FollowMaster);
    slaveGrabber.setInverted(true);

    drive.setRightSideInverted(false);
  }

  @Override
  public void teleopPeriodic() {
    // System.out.println("Switch: " + majElevatorDownSwitch.get());
    double forward = -stick.getY();
    double turn = stick.getX() * sensitivity;
    // Another sensitivity control. Controlled by a slider on the joystick. Value is
    // -1 ~ 1, the command converts it to 0 ~ 1.
    double sliderSensitivity = -(stick.getRawAxis(RobotMap.joySensitivitySlider) + 1) * 0.5;
    // System.out.println("JoyY:" + forward + " turn:" + turn );

    if (forward < 0) {
      forward *= 0.5;
    }
    // Cause Jing almost tipped the robot while driving back

    drive.arcadeDrive(forward * sliderSensitivity, turn * sliderSensitivity);

    if (stick.getRawButton(RobotMap.joyMajorElevatorUp)) {
      majTopFirstHit = true;
      // Result is inverted, due to connection to NO and Ground on the limit switch.
      if (!majElevatorTopSwitch.get()) {
        if (majTopFirstHit) {
          // If it is before the first hit, it will go up at a faster rate.
          majorElevatorMode = "topFirstHit";
        } else {
          // If repeated hit, but under same click, goes up slower.
          majorElevatorMode = "topRepeatedHit";
        }
      } else {
        if (majTopFirstHit) {
          // If hits top and continues to press, and first hit, sets to repeat. Idle.
          majorElevatorMode = "idle";
          majTopFirstHit = false;
        }
      }
    } else if (stick.getRawButton(RobotMap.joyMajorElevatorDown)) {
      // Result is not inverted, due to connection to NC and Ground on limit switch.
      if (majElevatorDownSwitch.get()) {
        majorElevatorMode = "reachBottom";
      } else {
        majorElevatorMode = "idle";
      }
    } else {
      majorElevatorMode = "idle";
      // Passive Lifting to prevent. Reuires Calibration
    }

    if (stick.getPOV() != 0 && stick.getPOV != 180) { // If no POV overrides given.
      if (stick.getRawButton(RobotMap.joyMinorElevatorUp)) {
        minTopFirstHit = true;
        // Check limit switch condition. Connect to NO and Ground for inverse.
        if (!minElevatorTopSwitch.get()) {
          if (minTopFirstHit) {
            // If it is before the first hit, it will go up at a faster rate.
            minorElevatorMode = "topFirstHit";
          } else {
            // If repeated hit, but under same click, goes up slower.
            minorElevatorMode = "topRepeatedHit";
          }
        } else {
          if (minTopFirstHit) {
            // If hits top and continues to press, and first hit, sets to repeat. Idle.
            minorElevatorMode = "idle";
            minTopFirstHit = false;
          }
        }
      } else if (stick.getRawButton(RobotMap.joyMinorElevatorDown)) {
        // Check connection. If NO and Ground should be inverse
        if (!minElevatorDownSwitch.get()) {
          minorElevatorMode = "reachBottom";
        } else {
          minorElevatorMode = "idle";
        }
      } else {
        minorElevatorMode = "idle";
        // Passive Lifting to prevent. Reuires Calibration
      }
    }
    // System.out.println(stick.getPOV());
    // POV Overrides for manual grabber control.
    else if (stick.getPOV() == 0) {
      // If 0, grabber manually moves tiny bit upwards.
      minorElevatorMode = "manualUp";
    } else if (stick.getPOV() == 180) {
      // If 180, grabber manually move tiny bit down.
      minorElevatorMode = "manualDown";
    }

    if (stick.getRawButton(RobotMap.joyShoot)) {
      mainGrabber.set(0.5);
    } else if (stick.getRawButton(RobotMap.joySucc)) {
      mainGrabber.set(-0.5); // Adjust positive / negative until matches
    } else {
      mainGrabber.set(0);
    }

    switch (majorElevatorMode) {
    case "idle":
      majorElevator.set(0.2);
    case "topFirstHit":
      majorElevator.set(1);
    case "topRepeatedHit":
      majorElevator.set(0.5);
    case "reachBottom":
      majorElevator.set(-1);
    }

    switch (minorElevatorMode) {
    case "idle":
      minorElevator.set(0.2);
    case "topFirstHit":
      minorElevator.set(1);
    case "topRepeatedHit":
      minorElevator.set(0.5);
    case "reachBottom":
      minorElevator.set(-1);
    case "manualUp":
      minorElevator.set(0.4);
    case "manualDown":
      minorElevator.set(-0.1);
    }
  }

}
