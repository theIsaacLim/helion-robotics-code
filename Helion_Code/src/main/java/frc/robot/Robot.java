/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;



import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;


import frc.robot.VisionPipeline;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final int k_ticks_per_rev = 1024;
  private static final double k_wheel_diameter = 4.0 / 12.0;
  private static final double k_max_velocity = 10;

  private static final int k_left_channel = 0;
  private static final int k_right_channel = 1;

  private static final int k_left_encoder_port_a = 0;
  private static final int k_left_encoder_port_b = 1;
  private static final int k_right_encoder_port_a = 2;
  private static final int k_right_encoder_port_b = 3;

  private static final int k_gyro_port = 0;

  private static final String k_path_name = "example";
  private SpeedController m_left_motor;
  private SpeedController m_right_motor;

  private Encoder m_left_encoder;
  private Encoder m_right_encoder;

  private AnalogGyro m_gyro;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;

  private Notifier m_follower_notifier;

  public static OI m_oi;

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;

	private VisionThread visionThread;
	private double centerX = 0.0;

  private final Object imgLock = new Object();


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_left_motor = new Spark(k_left_channel);
    m_right_motor = new Spark(k_right_channel);
    m_left_encoder = new Encoder(k_left_encoder_port_a, k_left_encoder_port_b);
    m_right_encoder = new Encoder(k_right_encoder_port_a, k_right_encoder_port_b);
    m_gyro = new AnalogGyro(k_gyro_port);

    m_oi = new OI();
    // chooser.addOption("My Auto", new MyAutoCommand());
    CameraServer.getInstance().startAutomaticCapture();
    SmartDashboard.putData("Auto mode", m_chooser);


    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new MyVisionPipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (imgLock) {
                centerX = r.x + (r.width / 2);
            }
        }
    });
    visionThread.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(m_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }
  private void followPath() {
   if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
     m_follower_notifier.stop();
   } else {
     double left_speed = m_left_follower.calculate(m_left_encoder.get());
     double right_speed = m_right_follower.calculate(m_right_encoder.get());
     double heading = m_gyro.getAngle();
     double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
     double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
     double turn =  0.8 * (-1.0/80.0) * heading_difference;
     m_left_motor.set(left_speed + turn);
     m_right_motor.set(right_speed - turn);
   }
 }

 /**
  * This function is called periodically during autonomous.
  */
 @Override
 public void autonomousPeriodic() {
 }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_follower_notifier.stop();
    m_left_motor.set(0);
    m_right_motor.set(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //RobotMap.myDrive.arcadeDrive(RobotMap.leftJoy.getY(),  RobotMap.rightJoy.getX());
    System.out.println("y: " + -RobotMap.leftJoy.getRawAxis(1));
    System.out.println("z: " + RobotMap.leftJoy.getRawAxis(0));
    //RobotMap.secondDrive.arcadeDrive(-RobotMap.leftJoy.getRawAxis(1), RobotMap.leftJoy.getRawAxis(0));
  }
}
