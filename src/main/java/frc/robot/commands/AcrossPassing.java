package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class AcrossPassing extends Command{
    private Shooter shooter;

    public AcrossPassing(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShroudDegrees(58);
        shooter.setShooterRPM(5300);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.zeroShroud();
    }
}
