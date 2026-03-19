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
    public static final double kShooterP = 2.4d;
    public static final double kShooterI = 0.0d;
    public static final double kShooterD = 0.2d;

    public static final double kShroudP = 1.75d;
    public static final double kShroudI = 0.0d;
    public static final double kShroudD = 0.0d;
    public static final double kShroudS = 0.5d;
  }
}
