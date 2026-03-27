package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class HeadingDrive extends Command {
  private final Drive drivetrain;
  private final DoubleSupplier leftX;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;
  private final DoubleSupplier rightY;

  // No longer using CTRE SwerveRequest -- we'll build ChassisSpeeds and
  // send them to the Drive subsystem via runVelocity().

  // PID controller for heading control
  private final PIDController headingController;

  // Deadband threshold for joysticks
  private static final double DEADBAND = 0.1;

  // Maximum speeds
  private final double maxSpeed;
  private final double maxAngularRate;

  // Target heading (maintained when joystick is centered)
  private Rotation2d targetHeading;
  // Last observed robot heading; used to detect abrupt resets to the
  // field-centric
  // reference (e.g. when drivetrain.seedFieldCentric(...) is called).
  private Rotation2d lastHeading;

  // Threshold (radians) to consider the heading jumped due to a reset. 45 deg.
  private static final double HEADING_RESET_THRESHOLD = Math.toRadians(45.0);
  // Aim joystick heading offset in degrees. Positive rotates counter-clockwise.
  private final double aimJoystickAngleOffsetDegrees;

  /**
   * Creates a new field-relative drive command with heading control
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param leftX Left joystick X (strafe)
   * @param leftY Left joystick Y (forward/back)
   * @param rightX Right joystick X (aiming)
   * @param rightY Right joystick Y (aiming)
   * @param maxSpeed Maximum translation speed (m/s)
   * @param maxAngularRate Maximum rotation rate (rad/s)
   */
  public HeadingDrive(
      Drive drivetrain,
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX,
      DoubleSupplier rightY,
      double maxSpeed,
      double maxAngularRate,
      double aimJoystickAngleOffsetDegrees) {

    this.drivetrain = drivetrain;
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.rightY = rightY;
    this.maxSpeed = maxSpeed;
    this.maxAngularRate = maxAngularRate;

    // Create PID controller for heading
    headingController = new PIDController(6.0, 0.001, 0.3);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(0.02); // ~1 degree tolerance

    this.aimJoystickAngleOffsetDegrees = aimJoystickAngleOffsetDegrees;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Start with current heading as target
    targetHeading = drivetrain.getRotation();
    lastHeading = targetHeading;
  }

  @Override
  public void execute() {
    // Get joystick inputs with deadband
    double xSpeed = applyDeadband(-leftY.getAsDouble(), DEADBAND) * maxSpeed;
    double ySpeed = applyDeadband(-leftX.getAsDouble(), DEADBAND) * maxSpeed;
    double rightJoyX = applyDeadband(rightX.getAsDouble(), DEADBAND);
    double rightJoyY = applyDeadband(-rightY.getAsDouble(), DEADBAND);

    // Calculate rotation speed
    double rotationSpeed;

    // Check if right joystick is being used
    double rightJoyMagnitude = Math.hypot(rightJoyX, rightJoyY);

    if (rightJoyMagnitude > 0.2) {
      // Right joystick is deflected - calculate desired heading
      // Apply configurable offset so joystick directions can be remapped (e.g.
      // make "right" map to robot-right by passing -90 degrees).
      var joystickHeading = new Rotation2d(rightJoyX, rightJoyY);
      if (aimJoystickAngleOffsetDegrees != 0.0) {
        joystickHeading =
            joystickHeading.rotateBy(new Rotation2d(Math.toRadians(aimJoystickAngleOffsetDegrees)));
      }
      targetHeading = joystickHeading;
    }

    // Use PID to rotate toward target heading
    Rotation2d currentHeading = drivetrain.getRotation();
    // Detect abrupt heading resets (for example, when seedFieldCentric is run).
    // If a large jump is observed and the user is not actively commanding
    // the right stick, update the stored targetHeading to the new heading
    // and reset the PID to avoid aggressive corrective motion.
    double rawDiff = currentHeading.getRadians() - lastHeading.getRadians();
    // normalize to [-PI, PI]
    while (rawDiff > Math.PI) rawDiff -= 2.0 * Math.PI;
    while (rawDiff < -Math.PI) rawDiff += 2.0 * Math.PI;
    double absDiff = Math.abs(rawDiff);
    if (absDiff > HEADING_RESET_THRESHOLD && rightJoyMagnitude <= 0.2) {
      targetHeading = currentHeading;
      headingController.reset();
    }

    rotationSpeed =
        headingController.calculate(currentHeading.getRadians(), targetHeading.getRadians());

    // remember last heading for next loop
    lastHeading = currentHeading;

    // Clamp rotation speed
    rotationSpeed = MathUtil.clamp(rotationSpeed, -maxAngularRate, maxAngularRate);

    // Build chassis speeds from field-relative inputs and send to drivetrain
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotationSpeed, drivetrain.getRotation());
    drivetrain.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.stop();
  }

  /** Apply deadband to joystick input */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
  }
}
