package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ThrottleLog;

public class Shooter extends SubsystemBase {
  private final ShooterIO mShooter;
  private final ThrottleLog tLog;

  public Shooter(ShooterIO shooter) {
    this.mShooter = shooter;
    this.tLog = new ThrottleLog(ShooterConstants.kLogInterval);
  }

  public Command shoot(double RPM) {
    return startEnd(() -> mShooter.setSpeedRPM(RPM), () -> mShooter.setSpeed(0));
  }

  public Command shootPrecent(double precent) {
    return startEnd(() -> mShooter.setSpeed(precent), () -> mShooter.setSpeed(0));
  }

  public Command shroud(double degrees) {
    return runOnce(() -> mShooter.setShroud(degrees));
  }

  public Command stepShroud(double degrees) {
    return runOnce(() -> mShooter.moveShroud(degrees));
  }

  // Direct control methods for AutoAimShooter (without returning commands)
  public void setShooterRPM(double rpm) {
    mShooter.setSpeedRPM(rpm);
  }

  public void stopShooter() {
    mShooter.setSpeed(0);
  }

  public void setShroudDegrees(double degrees) {
    mShooter.setShroud(degrees);
  }

  public void zeroShroud() {
    mShooter.setShroud(0);
  }

  public double getShroud() {
    return mShooter.getShroud();
  }

  public void zero() {
    mShooter.zero();
  }

  // Expose shooter RPMs for logging and data collection
  public double getLeaderRPM() {
    return mShooter.getLeaderRPM();
  }

  public double getFollowerRPM() {
    return mShooter.getFollowerRPM();
  }

  @Override
  public void periodic() {
    tLog.log(
        () -> {
          SmartDashboard.putNumber("Shooter Leader RPM", Math.round(mShooter.getLeaderRPM()));
          SmartDashboard.putNumber("Shooter Follower RPM", Math.round(mShooter.getFollowerRPM()));
          SmartDashboard.putNumber("Shooter Shroud Degrees", Math.round(mShooter.getShroud()));
          org.littletonrobotics.junction.Logger.recordOutput(
              "Shooter/Actual/LeaderRPM", mShooter.getLeaderRPM());
          org.littletonrobotics.junction.Logger.recordOutput(
              "Shooter/Actual/FollowerRPM", mShooter.getFollowerRPM());
          org.littletonrobotics.junction.Logger.recordOutput(
              "Shooter/Actual/ShroudDegrees", mShooter.getShroud());
        });
  }
}
