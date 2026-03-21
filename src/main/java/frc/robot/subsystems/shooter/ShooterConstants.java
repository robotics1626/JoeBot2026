package frc.robot.subsystems.shooter;

public final class ShooterConstants {

  /**
   * Positions in degrees.
   *
   * @apiNote Not Implemented.
   */
  public enum Positions {
    kUp(13),
    kPassing(45),
    kDown(0);

    double degrees;

    private Positions(double degrees) {
      this.degrees = degrees;
    }

    public double getDegrees() {
      return this.degrees;
    }
  }

  /** Motor IDs. */
  public final class Motors {
    public static final int kShooterLeader = 9;
    public static final int kShooterFollower = 12;

    public static final int kShroud = 17;
  }

  /** Motor PIDs. */
  public final class PID {
    public static final double kShooterP = 6.7d; // 5.5
    public static final double kShooterI = 0.0069d;
    public static final double kShooterD = 0.03d; // 0.151
    public static final double kShooterS = 0.05d;
    public static final double kShooterV = 0.13d;

    public static final double kShroudP = 1.75d;
    public static final double kShroudI = 0.0d;
    public static final double kShroudD = 0.0d;
    public static final double kShroudS = 0.5d;
  }

  public class Control {
    public static final double kStoppedSpeed = 0.0;
    public static final double kNoTargetRpm = 0.0;
    public static final double kNoStableTimestamp = -1.0;
    public static final double kTargetEpsilonRpm = 1e-6;
    public static final double kDashboardDefaultTargetRpm = 3500.0;
    public static final double kDashboardMaxTargetRpm = 5500.0;
  }

  public final class AutoAim {
    public static final double kVisionDistanceScale = 1.0d; // Tuning parameter for vision distance; may need to be adjusted based on real-world testing
    public static final double kVisionDistanceBiasMeters = 0.0d; // Tuning parameter for vision distance; may need to be adjusted based on real-world testing

    public static final double[] kDistanceMeters =
    {};

    public static final double[] kPivotPosition =
    {};

    public static final double[] kShooterTargetRpm =
    {};
  }
}
