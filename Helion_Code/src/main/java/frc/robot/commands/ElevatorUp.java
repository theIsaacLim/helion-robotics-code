package frc.robot.commands;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;

public class ElevatorUp extends Command {
    public static float motor_speed = 0;
    public ElevatorUp() {
      // Use requires() here to declare subsystem dependencies
      // eg. requires(chassis);
      requires(Elevator.java)
    }
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        RobotMap.elevatorLargeMotor.setSpeed(motor_speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        RobotMap.elevatorLargeMotor.setSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    }
    }