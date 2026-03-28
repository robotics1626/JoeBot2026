package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class MidPassing extends Command{
    private Shooter shooter;

    public MidPassing(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterRPM(4500);
        shooter.setShroudDegrees(56);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.zeroShroud();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
