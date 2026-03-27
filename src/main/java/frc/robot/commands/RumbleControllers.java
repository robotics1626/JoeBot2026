package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleControllers extends Command {
  private CommandXboxController operator = null;
  private CommandXboxController driver = null;

  public RumbleControllers(CommandXboxController operator, CommandXboxController driver) {
    this.operator = operator;
    this.driver = driver;
  }

  @Override
  public void initialize() {
    if (this.operator == null || this.driver == null)
      throw new NullPointerException("Expected non null controllers, got one or two.");
  }

  @Override
  public void execute() {
    operator.setRumble(RumbleType.kBothRumble, 1);
    driver.setRumble(RumbleType.kBothRumble, 1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    operator.setRumble(RumbleType.kBothRumble, 0);
    driver.setRumble(RumbleType.kBothRumble, 0);
  }
}
