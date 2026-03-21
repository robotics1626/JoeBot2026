package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO mIntake;

  public Intake(IntakeIO intake) {
    this.mIntake = intake;
  }

  public Command intake() {
    return startEnd(() -> mIntake.run(IntakeConstants.Speeds.k1), () -> mIntake.run(0));
  }

  public Command out() {
    return startEnd(() -> mIntake.run(-IntakeConstants.Speeds.k1), () -> mIntake.run(0));
  }
}
