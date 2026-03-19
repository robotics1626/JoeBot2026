package frc.robot.subsystems.shooter;

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
}
