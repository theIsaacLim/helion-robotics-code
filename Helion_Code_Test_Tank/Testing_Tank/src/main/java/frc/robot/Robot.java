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
import edu.wpi.first.wpilibj.Servo;

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
  private Joystick movStick;
  private Joystick gameStick;

  // Sensitivity for movement, constant.
  private double sensitivity;

  // Limit switches for the major elevator.
  private DigitalInput majElevatorTopSwitch;
  private DigitalInput majElevatorDownSwitch;

  // Boolean to control whether its the first hit of the major elevator or the
  // repeated successions.
  private boolean majTopFirstHit;

  // Camera alongside with the boolean controlling backwards movemet
  private UsbCamera frontCamera;
  private UsbCamera backCamera;
  private CameraServer server;
  private boolean facingBack;

  // Hatch and grabber release servos
  private Servo grabberRelease;
  private Servo hatchRelease;

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

    movStick = new Joystick(RobotMap.joyChannel);
    gameStick = new Joystick(RobotMap.gameChannel);

    sensitivity = 1;

    majElevatorTopSwitch = new DigitalInput(RobotMap.lSMajEleUp);
    majElevatorDownSwitch = new DigitalInput(RobotMap.lSMajEleDown);

    // minElevatorTopSwitch = new DigitalInput(RobotMap.lSMinEleUp);
    // minElevatorDownSwitch = new DigitalInput(RobotMap.lSMinEleDown);

    majTopFirstHit = true;
    // minTopFirstHit = true;

    // majorElevatorMode = "idle";
    // minorElevatorMode = "idle";

    server = CameraServer.getInstance();
    frontCamera = server.startAutomaticCapture(0);
    frontCamera.setResolution(320, 240);
    frontCamera.setFPS(15);
    backCamera = server.startAutomaticCapture(1);
    backCamera.setResolution(320, 240);
    backCamera.setFPS(15);

    grabberRelease = new Servo(RobotMap.grabberReleaseServo);
    hatchRelease = new Servo(RobotMap.hatchReleaseServo);

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
    slaveGrabber.setInverted(InvertType.OpposeMaster);

    majorElevator.setInverted(true);

    drive.setRightSideInverted(false);

    hatchRelease.setAngle(RobotMap.hatchServoIdleAngle);
  }

  public void testPeriodic() {
    System.out.println(-movStick.getRawAxis(RobotMap.joySensitivitySlider) + 1);
  }

  @Override
  public void autonomousInit() {
    grabberRelease.setAngle(RobotMap.grabberServoReleaseAngle);
    facingBack = true;
  }

  @Override
  public void autonomousPeriodic() {
    double forward;
    if(!facingBack){
      forward = -movStick.getY();
    }
    else{
      forward = -movStick.getY();
    }
    System.out.println(forward);
    double turn = movStick.getX() * sensitivity;
    // Another sensitivity control. Controlled by a slider on the joystick. Value is
    // -1 ~ 1, the command converts it to 0 ~ 1.
    double sliderSensitivity = -movStick.getRawAxis(RobotMap.joySensitivitySlider) + 1;

    // Commented during autonomous
    // if (forward < 0) {
    //   forward *= 0.5;
    // }
    // Cause Jing almost tipped the robot while driving back

    drive.arcadeDrive(forward * sliderSensitivity, turn * sliderSensitivity);
    // System.out.println(gameStick.getRawAxis(RobotMap.joyMajorElevator));
    if (gameStick.getRawAxis(RobotMap.joyMajorElevator) < -0.25) {
      // System.out.println("going up");
      majTopFirstHit = true;
      // Result is inverted, due to connection to NO and Ground on the limit switch.
      if (!majElevatorTopSwitch.get()) {
        if (majTopFirstHit) {
          // If it is before the first hit, it will go up at a faster rate.
          majorElevator.set(1);
        } else {
          // If repeated hit, but under same click, goes up slower.
          majorElevator.set(0.2);
        }
      } else {
        if (majTopFirstHit) {
          // If hits top and continues to press, and first hit, sets to repeat. Idle.
          majorElevator.set(0.1);
          majTopFirstHit = false;
        }
      }
    } else if (gameStick.getRawAxis(RobotMap.joyMajorElevator) > 0.25) {
      // Result of limit switch is not inverted, due to connection to NC and Ground on limit switch.
      if (majElevatorDownSwitch.get()) {
        majorElevator.set(-1);
      } else {
        majorElevator.set(0.1);
      }
    } else {
      majorElevator.set(0.1);
      // Passive Lifting to prevent. Reuires Calibration
    }

    if (gameStick.getPOV() != 0 && gameStick.getPOV() != 180) { // If no POV overrides given.
      if (gameStick.getRawAxis(RobotMap.joyMinorElevator) < -0.25) {
        minorElevator.set(1);
      } else if (gameStick.getRawAxis(RobotMap.joyMinorElevator) > 0.25) {
        minorElevator.set(-1);
      } else {
        minorElevator.set(0.1);
        // Passive Lifting to prevent. Reuires Calibration
      }
    } else if (gameStick.getPOV() == 0) {
      // If 0, grabber manually moves tiny bit upwards.
      minorElevator.set(0.1);
    } else if (gameStick.getPOV() == 180) {
      // If 180, grabber manually move tiny bit down.
      minorElevator.set(-0.1);
    }

    if (gameStick.getRawAxis(RobotMap.joyShootLeft) > 0.5 || gameStick.getRawAxis(RobotMap.joyShootRight) > 0.5) {
      mainGrabber.set(0.7);
    } else if (gameStick.getRawButton(RobotMap.joySuccLeft) || gameStick.getRawButton(RobotMap.joySuccRight)) {
      mainGrabber.set(-0.3); // Adjust positive / negative until matches
    } else {
      mainGrabber.set(0);
    }

    if (gameStick.getRawButton(RobotMap.hatchServoRelease)) {
      hatchRelease.setAngle(RobotMap.hatchServoReleaseAngle);
    } else if (gameStick.getRawButton(RobotMap.hatchServoGrab)) {
      hatchRelease.setAngle(RobotMap.hatchServoGrabAngle);
    }
  }

  public void teleopInit(){
    facingBack = false;
  }

  @Override
  public void teleopPeriodic() {
    double forward;
    if(!facingBack){
      forward = -movStick.getY();
    }
    else{
      forward = -movStick.getY();
    }
    System.out.println(forward);
    double turn = movStick.getX() * sensitivity;
    // Another sensitivity control. Controlled by a slider on the joystick. Value is
    // -1 ~ 1, the command converts it to 0 ~ 1.
    double sliderSensitivity = -movStick.getRawAxis(RobotMap.joySensitivitySlider) + 1;

    if (forward < 0) {
      forward *= 0.8;
    }
    // Cause Jing almost tipped the robot while driving back

    drive.arcadeDrive(forward * sliderSensitivity, turn * sliderSensitivity);
    // System.out.println(gameStick.getRawAxis(RobotMap.joyMajorElevator));
    if (gameStick.getRawAxis(RobotMap.joyMajorElevator) < -0.25) {
      // System.out.println("going up");
      majTopFirstHit = true;
      // Result is inverted, due to connection to NO and Ground on the limit switch.
      if (!majElevatorTopSwitch.get()) {
        if (majTopFirstHit) {
          // If it is before the first hit, it will go up at a faster rate.
          majorElevator.set(1);
        } else {
          // If repeated hit, but under same click, goes up slower.
          majorElevator.set(0.2);
        }
      } else {
        if (majTopFirstHit) {
          // If hits top and continues to press, and first hit, sets to repeat. Idle.
          majorElevator.set(0.1);
          majTopFirstHit = false;
        }
      }
    } else if (gameStick.getRawAxis(RobotMap.joyMajorElevator) > 0.25) {
      // Result of limit switch is not inverted, due to connection to NC and Ground on limit switch.
      if (majElevatorDownSwitch.get()) {
        majorElevator.set(-1);
      } else {
        majorElevator.set(0.1);
      }
    } else {
      majorElevator.set(0.1);
      // Passive Lifting to prevent. Reuires Calibration
    }

    if (gameStick.getPOV() != 0 && gameStick.getPOV() != 180) { // If no POV overrides given.
      if (gameStick.getRawAxis(RobotMap.joyMinorElevator) < -0.25) {
        minorElevator.set(1);
      } else if (gameStick.getRawAxis(RobotMap.joyMinorElevator) > 0.25) {
        minorElevator.set(-1);
      } else {
        minorElevator.set(0.1);
        // Passive Lifting to prevent. Reuires Calibration
      }
    } else if (gameStick.getPOV() == 0) {
      // If 0, grabber manually moves tiny bit upwards.
      minorElevator.set(0.1);
    } else if (gameStick.getPOV() == 180) {
      // If 180, grabber manually move tiny bit down.
      minorElevator.set(-0.1);
    }

    if (gameStick.getRawAxis(RobotMap.joyShootLeft) > 0.5 || gameStick.getRawAxis(RobotMap.joyShootRight) > 0.5) {
      mainGrabber.set(0.7);
    } else if (gameStick.getRawButton(RobotMap.joySuccLeft) || gameStick.getRawButton(RobotMap.joySuccRight)) {
      mainGrabber.set(-0.5); // Adjust positive / negative until matches
    } else {
      mainGrabber.set(0);
    }

    if (gameStick.getRawButton(RobotMap.hatchServoRelease)) {
      hatchRelease.setAngle(RobotMap.hatchServoReleaseAngle);
    } else if (gameStick.getRawButton(RobotMap.hatchServoGrab)) {
      hatchRelease.setAngle(RobotMap.hatchServoGrabAngle);
    }
  }
}
