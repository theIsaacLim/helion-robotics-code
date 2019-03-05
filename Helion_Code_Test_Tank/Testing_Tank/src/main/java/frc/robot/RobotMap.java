package frc.robot;

public class RobotMap {
    public static final int flChannel = 0;
    public static final int frChannel = 2;
    public static final int blChannel = 1;
    public static final int brChannel = 3;

    public static final int majElevatorChannel = 4;
    public static final int minElevatorChannel = 5;

    public static final int mainGrabber = 6;
    public static final int slaveGrabber = 7;

    public static final int joyChannel = 0;
    public static final int joySensitivitySlider = 3;

    public static final int gameChannel = 1; // Gamepad channel. Controls elevator. The rest on the bottom should be adjusted based on gamepad preferences.
    public static final int joyShoot = 1;
    public static final int joySucc = 2;
    public static final int joyMajorElevatorUp = 5;
    public static final int joyMajorElevatorDown = 3;
    public static final int joyMinorElevatorUp = 6;
    public static final int joyMinorElevatorDown = 4;
    public static final int hatchServoRelease = 7;
    public static final int hatchServoGrab = 8;

    //DIO
    public static final int lSMajEleUp = 0;
    public static final int lSMajEleDown = 1;
    public static final int lSMinEleUp = 2;
    public static final int lSMinEleDown = 3;

    //PWM
    public static final int grabberReleaseServo = 0;
    public static final int hatchReleaseServo = 1;

    //Servo Values
    public static final double grabberServoReleaseAngle = 50; //Must test
    public static final double hatchServoReleaseAngle = 50;
    public static final double hatchServoGrabAngle = 50;
    public static final double hatchServoIdleAngle = 50;
}