// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlignHeadingToHub;
import frc.robot.commands.AutoAimShooter;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HeadingDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalon;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalon;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Shooter shooter = new Shooter(new ShooterIOTalon());
  private final Intake intake = new Intake(new IntakeIOSpark());
  private final Indexer indexer = new Indexer(new IndexerIOTalon());
  //   private final AutoAimShooter aimbot;

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(15).in(RadiansPerSecond); // No clue lol

  public DoubleSupplier DebugRPMValue = () -> 1000;

  // Testing variables for incremental shooter control
  public double testShooterRPM = 3000.0;
  private double testShroudDegrees = 0.0;
  private static final double RPM_INCREMENT = 100.0;
  private static final double SHROUD_INCREMENT = 1.0;
  private static final double RPM_MIN = 0.0;
  private static final double RPM_MAX = 6000.0;
  private static final double SHROUD_MIN = 0.0;
  private static final double SHROUD_MAX = 45.0;

  // Joystick deadbands
  private static final double LEFT_JOYSTICK_DEADBAND = 0.2;
  private static final double RIGHT_JOYSTICK_DEADBAND = 0.5;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("GiggleCam", VisionConstants.robotToCamera0));

        // aimbot = new AutoAimShooter(drive, vision, shooter);
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "GiggleCam", VisionConstants.robotToCamera0, drive::getPose));

        // aimbot = new AutoAimShooter(drive, vision, shooter);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        // aimbot = new AutoAimShooter(drive, vision, shooter);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    NamedCommands.registerCommand("AutoAimShooter", new AutoAimShooter(drive, vision, shooter));
    NamedCommands.registerCommand(
        "AlignHeadingToHub",
        new AlignHeadingToHub(
            drive,
            () -> -applyLeftDeadband(driver.getLeftY()),
            () -> -applyLeftDeadband(driver.getLeftX()),
            true));
    NamedCommands.registerCommand("IndexFlow", indexer.indexFlow());
    // Configure the button bindings
    // Use configureButtonBindings() for two-controller setup (default)
    // Use configureButtonBindingsOneController() for single-controller setup
    // configureButtonBindings();
    configureButtonBindingsOneController();
  }

  /**
   * Apply deadband to left joystick input (0.2 deadband). Values within the deadband are set to 0,
   * values outside are scaled from 0 to 1.
   */
  private double applyLeftDeadband(double value) {
    if (Math.abs(value) < LEFT_JOYSTICK_DEADBAND) {
      return 0.0;
    }
    // Scale from [deadband, 1] to [0, 1]
    return Math.copySign(
        (Math.abs(value) - LEFT_JOYSTICK_DEADBAND) / (1.0 - LEFT_JOYSTICK_DEADBAND), value);
  }

  /**
   * Apply deadband to right joystick input (0.5 deadband). Values within the deadband are set to 0,
   * values outside are scaled from 0 to 1.
   */
  private double applyRightDeadband(double value) {
    if (Math.abs(value) < RIGHT_JOYSTICK_DEADBAND) {
      return 0.0;
    }
    // Scale from [deadband, 1] to [0, 1]
    return Math.copySign(
        (Math.abs(value) - RIGHT_JOYSTICK_DEADBAND) / (1.0 - RIGHT_JOYSTICK_DEADBAND), value);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () ->
    // -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -applyLeftDeadband(driver.getLeftY()),
                () -> -applyLeftDeadband(driver.getLeftX()),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    RobotModeTriggers.teleop()
        .whileTrue(
            new HeadingDrive(
                drive,
                () -> applyLeftDeadband(driver.getLeftX()),
                () -> applyLeftDeadband(driver.getLeftY()),
                () -> -applyRightDeadband(driver.getRightX()),
                () -> -applyRightDeadband(driver.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true))
        .onTrue(
            new HeadingDrive(
                drive,
                () -> applyLeftDeadband(driver.getLeftX()),
                () -> applyLeftDeadband(driver.getLeftY()),
                () -> -applyRightDeadband(driver.getRightX()),
                () -> -applyRightDeadband(driver.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

    // driver.rightTrigger().whileTrue(aimbot);
    // Right trigger: align to hub using AlignToPose
    driver
        .rightTrigger()
        .whileTrue(
            new AlignHeadingToHub(
                    drive,
                    () -> -applyLeftDeadband(driver.getLeftY()),
                    () -> -applyLeftDeadband(driver.getLeftX()),
                    true)
                .alongWith(new AutoAimShooter(drive, vision, shooter)))
        .onFalse(
            new HeadingDrive(
                drive,
                () -> applyLeftDeadband(driver.getLeftX()),
                () -> applyLeftDeadband(driver.getLeftY()),
                () -> -applyRightDeadband(driver.getRightX()),
                () -> -applyRightDeadband(driver.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

    /// Operator
    operator.b().whileTrue(indexer.index().alongWith(intake.intake()));

    operator.a().whileTrue(shooter.shootPrecent(1));

    // operator.rightTrigger().whileTrue(shooter.shoot(3000));
    // operator.rightTrigger().whileTrue(new AutoAimShooter(drive, vision, shooter));

    operator.x().whileTrue(shooter.shoot(4000));
    operator.y().whileTrue(shooter.shoot(6000));

    operator.leftBumper().whileTrue(indexer.ohShit().alongWith(intake.out()));

    operator.leftTrigger().whileTrue(indexer.indexFlow().alongWith(intake.intake()));

    // make the justIndexer and the feed command run together w/ out the feedtoshooter command

    operator.rightBumper().whileTrue(indexer.feedRunOnce()).onFalse(indexer.stopFeedRunOnce());
    operator
        .rightBumper()
        .whileTrue(indexer.justIndexerRunOnce())
        .onFalse(indexer.stopJustIndexerRunOnce());

    operator.povUp().onTrue(shooter.stepShroud(5));

    operator.povDown().onTrue(shooter.stepShroud(-5));

    operator.povLeft().onTrue(shooter.shroud(0));

    operator.povRight().onTrue(shooter.shroud(13));

    // Testing buttons: increment/decrement shooter RPM
    operator
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testShooterRPM = Math.min(testShooterRPM + RPM_INCREMENT, RPM_MAX);
                  shooter.shoot(testShooterRPM).schedule();
                }));

    operator
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testShooterRPM = Math.max(testShooterRPM - RPM_INCREMENT, RPM_MIN);
                  shooter.shoot(testShooterRPM).schedule();
                }));

    // Testing buttons: increment/decrement shroud angle

  }

  /**
   * Configure button bindings for single-controller operation. One operator controls both drive and
   * all subsystems (shooter, indexer, intake).
   *
   * <p>Control Scheme:
   *
   * <ul>
   *   <li>Left Stick X/Y: Drive translation (forward/backward, left/right)
   *   <li>Right Stick X: Drive rotation
   *   <li>Right Stick Y: Shooter RPM selection (push up for higher RPM)
   *   <li>A Button: Shoot at current setpoint
   *   <li>B Button: Index + Intake
   *   <li>X Button: Stop with X pattern
   *   <li>Y Button: Rotate to 0° heading
   *   <li>Left Bumper: Reverse (ohShit + intake out)
   *   <li>Left Trigger: Index flow + intake
   *   <li>Right Bumper: Feed to shooter
   *   <li>Right Trigger: Align to hub + auto aim
   *   <li>POV Up/Down: Increase/decrease shroud angle
   *   <li>POV Left: Set shroud to 0°
   *   <li>POV Right: Set shroud to 13°
   *   <li>Start/Back: Increment/decrement shroud angle (alternative)
   * </ul>
   */
  private void configureButtonBindingsOneController() {
    CommandXboxController solo = driver; // Reuse driver controller for one-controller mode

    // Default command: field-relative drive with right stick rotation
    solo.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    RobotModeTriggers.teleop()
        .whileTrue(
            new HeadingDrive(
                drive,
                () -> applyLeftDeadband(solo.getLeftX()),
                () -> applyLeftDeadband(solo.getLeftY()),
                () -> -applyRightDeadband(solo.getRightX()),
                () -> -applyRightDeadband(solo.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

    // A Button: Shoot at current test RPM
    solo.a()
        .whileTrue(shooter.shoot(testShooterRPM).alongWith(indexer.feed()))
        .onFalse(indexer.stopFeedRunOnce());

    // B Button: Index + Intake
    solo.b().whileTrue(indexer.index().alongWith(intake.intake()));

    // Y Button: Reset gyro to 0°
    solo.y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true))
        .onTrue(
            new HeadingDrive(
                drive,
                () -> applyLeftDeadband(solo.getLeftX()),
                () -> applyLeftDeadband(solo.getLeftY()),
                () -> -applyRightDeadband(solo.getRightX()),
                () -> -applyRightDeadband(solo.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

    // Right Trigger: Align to hub + auto aim
    solo.rightTrigger()
        .whileTrue(
            new AlignHeadingToHub(
                    drive,
                    () -> -applyLeftDeadband(solo.getLeftY()),
                    () -> -applyLeftDeadband(solo.getLeftX()),
                    true)
                .alongWith(new AutoAimShooter(drive, vision, shooter)))
        .onFalse(
            new HeadingDrive(
                drive,
                () -> applyLeftDeadband(solo.getLeftX()),
                () -> applyLeftDeadband(solo.getLeftY()),
                () -> -applyRightDeadband(solo.getRightX()),
                () -> -applyRightDeadband(solo.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

    // Left Bumper: Reverse all (ohShit + intake out)
    solo.leftBumper().whileTrue(indexer.ohShit().alongWith(intake.out()));

    // Left Trigger: Index flow + intake
    solo.leftTrigger().whileTrue(indexer.indexFlow().alongWith(intake.intake()));

    // Right Bumper: Feed to shooter
    solo.rightBumper().whileTrue(indexer.feedRunOnce()).onFalse(indexer.stopFeedRunOnce());
    solo.rightBumper()
        .whileTrue(indexer.justIndexerRunOnce())
        .onFalse(indexer.stopJustIndexerRunOnce());

    // POV Controls: Shooter shroud angle
    solo.povUp().onTrue(shooter.stepShroud(5));
    solo.povDown().onTrue(shooter.stepShroud(-5));
    solo.povLeft().onTrue(shooter.shroud(0));
    solo.povRight().onTrue(shooter.shroud(13));

    // Start Button: Increment shooter RPM
    solo.start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testShooterRPM = Math.min(testShooterRPM + RPM_INCREMENT, RPM_MAX);
                  Logger.recordOutput("OneController/TestShooterRPM", testShooterRPM);
                }));

    // Back Button: Decrement shooter RPM
    solo.back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testShooterRPM = Math.max(testShooterRPM - RPM_INCREMENT, RPM_MIN);
                  Logger.recordOutput("OneController/TestShooterRPM", testShooterRPM);
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get the auto command without resetting pose - this keeps current odometry
    Command autoCommand = autoChooser.get();

    // Return the auto command as-is (it will use the current pose as starting point)
    return autoCommand;
  }
}
