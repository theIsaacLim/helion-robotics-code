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
    public static final int joyShootLeft = 2; //Is a trigger channel, technically a slide. Needs code change
    public static final int joyShootRight = 3; //Same as above
    public static final int joySuccLeft = 5;
    public static final int joySuccRight = 6;
    // public static final int joyMajorElevatorUp = 5;
    // public static final int joyMajorElevatorDown = 3;
    // public static final int joyMinorElevatorUp = 6;
    // public static final int joyMinorElevatorDown = 4;
    public static final int joyMajorElevator = 1; //Left stick controls up and down of major elevator
    public static final int joyMinorElevator = 5; //Same as above
    public static final int hatchServoRelease = 1; //These are buttons
    public static final int hatchServoGrab = 2;
    public static final int cameraSwitch = 3;

    //DIO
    public static final int lSMajEleUp = 0;
    public static final int lSMajEleDown = 1;
    public static final int lSMinEleUp = 2;
    public static final int lSMinEleDown = 3;

    //PWM
    public static final int grabberReleaseServo = 0;
    public static final int hatchReleaseServo = 1;

    //Servo Values
    public static final double grabberServoReleaseAngle = 180; //Must test
    public static final double hatchServoReleaseAngle = 20;
    public static final double hatchServoGrabAngle = 60;
    public static final double hatchServoIdleAngle = 90;
}