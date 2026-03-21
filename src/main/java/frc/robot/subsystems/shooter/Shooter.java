package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO mShooter;

  public Shooter(ShooterIO shooter) {
    this.mShooter = shooter;
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

  public double getShroud() {
    return mShooter.getShroud();
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
    SmartDashboard.putNumber("Shooter Leader RPM", Math.round(mShooter.getLeaderRPM()));
    SmartDashboard.putNumber("Shooter Follower RPM", Math.round(mShooter.getFollowerRPM()));
    SmartDashboard.putNumber("Shooter Shroud Degrees", Math.round(mShooter.getShroud()));

    // Also record to structured logger for dataset collection
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Actual/LeaderRPM", mShooter.getLeaderRPM());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Actual/FollowerRPM", mShooter.getFollowerRPM());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Actual/ShroudDegrees", mShooter.getShroud());
  }
}
