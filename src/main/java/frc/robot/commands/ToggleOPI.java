package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleOPI extends Command {
  private PowerDistribution pdh;

  public ToggleOPI() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public void execute() {
    pdh.setSwitchableChannel(false);
  }

  @Override
  public boolean isFinished() {
    if (pdh.getSwitchableChannel() == true) return false;
    else return true;
  }

  @Override
  public void end(boolean interrupted) {
    pdh.setSwitchableChannel(true);
  }
}
