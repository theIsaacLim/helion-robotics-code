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

  // Boolean to control whether its the first hit of the elevator or the repeated
  // successions.
  private boolean topFirstHit;

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
      topFirstHit = true;
      // Result is inverted, due to connection to NO and Ground on the limit switch.
      if (!majElevatorTopSwitch.get()) {
        if (topFirstHit) {
          // If it is before the first hit, it will go up at a faster rate.
          majorElevator.set(1);
        } else {
          // If repeated hit, but under same click, goes up slower.
          majorElevator.set(0.5);
        }
      } else {
        if (topFirstHit) {
          // If it is not clicked, and it is after first hit. Sets it back to the first
          // click.
          topFirstHit = false;
        }
        majorElevator.set(0.2);
        // Passive Lifting to prevent. Reuires Calibration
      }
    } else if (stick.getRawButton(RobotMap.joyMajorElevatorDown)) {
      // Result is not inverted, due to connection to NC and Ground on limit switch.
      if (majElevatorDownSwitch.get()) {
        majorElevator.set(-1);
      } else {
        majorElevator.set(0);
      }
    } else {
      majorElevator.set(0);
    }

    if (stick.getRawButton(RobotMap.joyMinorElevatorUp)) {
      minorElevator.set(1);
    } else if (stick.getRawButton(RobotMap.joyMinorElevatorDown)) {
      minorElevator.set(-1);
    } else {
      minorElevator.set(0);
    }
    // System.out.println(stick.getPOV());

    if (stick.getRawButton(RobotMap.joyShoot)) {
      mainGrabber.set(0.5);
    } else if (stick.getRawButton(RobotMap.joySucc)) {
      mainGrabber.set(-0.5); // Adjust until matches
    } else {
      mainGrabber.set(0);
    }
  }

}
