package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ThrottleLog;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously aims the shooter pivot and wheel speed from distance-to-hub interpolation tables.
 */
public class AutoAimShooter extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;

  private final InterpolatingDoubleTreeMap pivotMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRpmMap = new InterpolatingDoubleTreeMap();

  private final double minDistanceMeters;
  private final double maxDistanceMeters;

  private final ThrottleLog tLog = new ThrottleLog(10);

  public AutoAimShooter(Drive drive, Vision vision, Shooter shooter) {
    this.drive = drive;
    this.vision = vision;
    this.shooter = shooter;

    for (int i = 0; i < ShooterConstants.AutoAim.kDistanceMeters.length; i++) {
      double distance = ShooterConstants.AutoAim.kDistanceMeters[i];
      pivotMap.put(distance, ShooterConstants.AutoAim.kPivotPosition[i]);
      shooterRpmMap.put(distance, ShooterConstants.AutoAim.kShooterTargetRpm[i]);
    }

    minDistanceMeters = ShooterConstants.AutoAim.kDistanceMeters[0];
    maxDistanceMeters =
        ShooterConstants.AutoAim.kDistanceMeters[
            ShooterConstants.AutoAim.kDistanceMeters.length - 1];

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    OptionalDouble visionDistanceMeters = vision.getLatestHubDistanceMeters();

    Pose2d robotPose = drive.getPose();
    Translation2d hubCenter =
        AllianceFlipUtil.apply(FieldConstants.Hub.blueCenter).getTranslation();
    double odometryDistanceMeters = robotPose.getTranslation().getDistance(hubCenter);

    boolean usingVisionDistance = visionDistanceMeters.isPresent();
    double rawVisionDistanceMeters = visionDistanceMeters.orElse(Double.NaN);
    double adjustedVisionDistanceMeters =
        usingVisionDistance
            ? rawVisionDistanceMeters * ShooterConstants.AutoAim.kVisionDistanceScale
                + ShooterConstants.AutoAim.kVisionDistanceBiasMeters
            : Double.NaN;

    double distanceMeters =
        usingVisionDistance ? adjustedVisionDistanceMeters : odometryDistanceMeters;
    double clampedDistance = MathUtil.clamp(distanceMeters, minDistanceMeters, maxDistanceMeters);

    double pivotSetpoint = pivotMap.get(clampedDistance);

    double shooterTargetRpm = shooterRpmMap.get(clampedDistance);

    // Directly set shooter values (don't use Commands since this command already owns shooter)
    shooter.setShroudDegrees(pivotSetpoint + 7);

    // Clamp shooter RPM to valid range
    double clampedRpm =
        MathUtil.clamp(shooterTargetRpm, 0.0, ShooterConstants.Control.kDashboardMaxTargetRpm);
    shooter.setShooterRPM(clampedRpm * 1.030);
    tLog.log(
        () -> {
          // Log core fields used for creating interpolation tables
          Logger.recordOutput("Shooter/AutoAim/Timestamp", Timer.getFPGATimestamp());
          Logger.recordOutput("Shooter/AutoAim/UsingVisionDistance", usingVisionDistance);
          Logger.recordOutput("Shooter/AutoAim/RawVisionDistanceMeters", rawVisionDistanceMeters);
          Logger.recordOutput(
              "Shooter/AutoAim/AdjustedVisionDistanceMeters", adjustedVisionDistanceMeters);
          Logger.recordOutput("Shooter/AutoAim/OdometryDistanceMeters", odometryDistanceMeters);
          Logger.recordOutput("Shooter/AutoAim/DistanceMeters", distanceMeters);
          Logger.recordOutput("Shooter/AutoAim/DistanceClampedMeters", clampedDistance);
          Logger.recordOutput("Shooter/AutoAim/PivotSetpoint", pivotSetpoint);
          Logger.recordOutput("Shooter/AutoAim/PivotEncoderPosition", shooter.getShroud());
          Logger.recordOutput("Shooter/AutoAim/RecommendedShooterTargetRpm", shooterTargetRpm);
          Logger.recordOutput("Shooter/AutoAim/RecommendedPivotPosition", pivotSetpoint);

          // Log actual shooter state
          Logger.recordOutput("Shooter/AutoAim/ActualLeaderRpm", shooter.getLeaderRPM());
          Logger.recordOutput("Shooter/AutoAim/ActualFollowerRpm", shooter.getFollowerRPM());

          // Log robot pose (useful to correlate distance with field position)
          Logger.recordOutput("Shooter/AutoAim/RobotPoseX", robotPose.getX());
          Logger.recordOutput("Shooter/AutoAim/RobotPoseY", robotPose.getY());
          Logger.recordOutput(
              "Shooter/AutoAim/RobotPoseHeadingDegrees", robotPose.getRotation().getDegrees());
        });

    // Log latest vision target details if present
    var latestObs = vision.getLatestTargetObservation();
    if (latestObs.isPresent()) {
      var obs = latestObs.get();
      tLog.log(
          () -> {
            Logger.recordOutput("Shooter/AutoAim/LatestTargetId", obs.tagId());
            Logger.recordOutput("Shooter/AutoAim/LatestTargetTxDeg", obs.tx().getDegrees());
            Logger.recordOutput("Shooter/AutoAim/LatestTargetTyDeg", obs.ty().getDegrees());
            Logger.recordOutput("Shooter/AutoAim/LatestTargetDistanceMeters", obs.distanceMeters());
          });
    } else {
      tLog.log(
          () -> {
            Logger.recordOutput("Shooter/AutoAim/LatestTargetId", -1);
            Logger.recordOutput("Shooter/AutoAim/LatestTargetTxDeg", Double.NaN);
            Logger.recordOutput("Shooter/AutoAim/LatestTargetTyDeg", Double.NaN);
            Logger.recordOutput("Shooter/AutoAim/LatestTargetDistanceMeters", Double.NaN);
          });
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.zero();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // public Command shootertuned(double shooterTargetRpm) {
  //   return Commands.parallel(
  //       shooter.setShooterSpeedWithTargetRpm(ShooterConstants.Top.kOutSpeed, shooterTargetRpm),
  //       Commands.sequence(Commands.waitSeconds(1.0), feeder.feedFuel()));
  // }
}
