package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.utilities.Drive;

public class DriveSub extends Subsystem {

  TalonSRX rf, rb, lf, lb;
  Drive drive;

  public DriveSub(){
    rf = new TalonSRX(RobotMap.frontRightChannel);
    rb = new TalonSRX(RobotMap.backRightChannel);
    lf = new TalonSRX(RobotMap.frontLeftChannel);
    lb = new TalonSRX(RobotMap.backLeftChannel);

    lf.setInverted(true);
    lb.setInverted(true);

    rb.set(ControlMode.Follower, rf.getDeviceID());
    lb.set(ControlMode.Follower, lf.getDeviceID());

    ConfigTalons(rf);
    ConfigTalons(rb);
    ConfigTalons(lf);
    ConfigTalons(lb);

    drive = new Drive(lf, rf);
  }

  public void ConfigTalons(TalonSRX tSRX){
    tSRX.configPeakOutputForward(1, 0);
    tSRX.configPeakOutputReverse(-1, 0);

    tSRX.configPeakCurrentLimit(40, 0);
    tSRX.enableCurrentLimit(true);
    tSRX.configContinuousCurrentLimit(40, 0);

    tSRX.configPeakCurrentDuration(250, 0);
    tSRX.configVoltageCompSaturation(12, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
