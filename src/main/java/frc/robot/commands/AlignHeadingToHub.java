package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ThrottleLog;
import java.util.function.DoubleSupplier;

/** Aligns the robot's heading to face the hub while allowing the driver to translate. */
public class AlignHeadingToHub extends Command {
  private final Drive drive;
  private final PIDController headingController;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private boolean hinter;

  private ThrottleLog tLog = new ThrottleLog(10);

  public AlignHeadingToHub(
      Drive drive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, boolean hinter) {
    this.drive = drive;
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;

    // Simple PID for heading control
    this.headingController = new PIDController(15.0, 5.0, 2.0);
    this.headingController.enableContinuousInput(-Math.PI, Math.PI);

    this.hinter = hinter;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Get robot pose
    Pose2d robotPose = drive.getPose();

    // Get hub center (alliance-aware)
    Pose2d hubPose = AllianceFlipUtil.apply(FieldConstants.Hub.blueCenter);

    // Calculate angle to hub
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();
    double targetYaw = Math.atan2(dy, dx);

    // Calculate distance to hub
    double distanceToHub = Math.sqrt(dx * dx + dy * dy);

    // Log distance to SmartDashboard
    tLog.log(
        () -> {
          SmartDashboard.putNumber("AlignHeadingToHub/DistanceToHub", distanceToHub);
        });

    // Calculate rotation command
    double rotationSpeed =
        headingController.calculate(robotPose.getRotation().getRadians(), targetYaw);
    rotationSpeed =
        MathUtil.clamp(
            rotationSpeed,
            -drive.getMaxAngularSpeedRadPerSec(),
            drive.getMaxAngularSpeedRadPerSec());

    // HAZAAA BAZOOKA

    // Get driver translation inputs
    double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
    double hinteredSpeed = 1.1;
    double xSpeed = forwardSupplier.getAsDouble() * (hinter ? hinteredSpeed : maxSpeed);
    double ySpeed = strafeSupplier.getAsDouble() * (hinter ? hinteredSpeed : maxSpeed);

    // Build field-relative speeds
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, drive.getRotation());

    // Command the drive
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until trigger released
  }
}
