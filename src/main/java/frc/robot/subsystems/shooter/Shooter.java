package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ThrottleLog;

public class Shooter extends SubsystemBase {
  private final ShooterIO mShooter;
  private final ThrottleLog tLog;
  private double targetRPM = 0.0;

  public Shooter(ShooterIO shooter) {
    this.mShooter = shooter;
    this.tLog = new ThrottleLog(ShooterConstants.kLogInterval);
  }

  public Command shoot(double RPM) {
    return startEnd(() -> setShooterRPM(RPM), this::stopShooter);
  }

  public Command shootPrecent(double precent) {
    return startEnd(
        () -> {
          targetRPM = 0.0;
          mShooter.setSpeed(precent);
        },
        this::stopShooter);
  }

  public Command shroud(double degrees) {
    return runOnce(() -> mShooter.setShroud(degrees));
  }

  public Command stepShroud(double degrees) {
    return runOnce(() -> mShooter.moveShroud(degrees));
  }

  // Direct control methods for AutoAimShooter (without returning commands)
  public void setShooterRPM(double rpm) {
    targetRPM = rpm;
    mShooter.setSpeedRPM(rpm);
  }

  public void stopShooter() {
    targetRPM = 0.0;
    mShooter.setSpeed(0);
  }

  /** Last RPM commanded with {@link #setShooterRPM}, or 0 when stopped. */
  public double getTargetRPM() {
    return targetRPM;
  }

  /** True when a velocity target is set and both flywheels are within tolerance of it. */
  public boolean isAtTargetRPM() {
    double tolerance = ShooterConstants.Control.kAtTargetToleranceRpm;
    return targetRPM > 0.0
        && Math.abs(Math.abs(getLeaderRPM()) - targetRPM) < tolerance
        && Math.abs(Math.abs(getFollowerRPM()) - targetRPM) < tolerance;
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
    targetRPM = 0.0;
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
