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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AcrossPassing;
import frc.robot.commands.AlignHeadingToHub;
import frc.robot.commands.AutoAimShooter;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HeadingDrive;
import frc.robot.commands.MidPassing;
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

import java.lang.management.OperatingSystemMXBean;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;

    // @SuppressWarnings("unused") //
    private final Vision vision;

    private final Shooter shooter = new Shooter(new ShooterIOTalon());
    private final Intake intake = new Intake(new IntakeIOSpark());
    private final Indexer indexer = new Indexer(new IndexerIOTalon());
    // private final AutoAimShooter aimbot;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(15).in(RadiansPerSecond);

    public DoubleSupplier DebugRPMValue = () -> 1000;

    // Testing variables for incremental shooter control
    public double testShooterRPM = 3000.0;
    private static final double RPM_INCREMENT = 100.0;
    private static final double RPM_MIN = 0.0;
    private static final double RPM_MAX = 6000.0;

    // Joystick deadbands
    private static final double LEFT_JOYSTICK_DEADBAND = 0.2;
    private static final double RIGHT_JOYSTICK_DEADBAND = 0.5;

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final AlignHeadingToHub autoAlignHeadingToHub;
    private final AutoAimShooter auto_autoAimShooter;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                vision = new Vision(
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
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                // Sim robot, instantiate physics sim IO implementations
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(
                                "GiggleCam", VisionConstants.robotToCamera0, drive::getPose));

                // aimbot = new AutoAimShooter(drive, vision, shooter);
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                // (Use same number of dummy implementations as the real robot)
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                // aimbot = new AutoAimShooter(drive, vision, shooter);
                break;
        }

        NamedCommands.registerCommand("AutoAimShooter", new AutoAimShooter(drive, vision, shooter));
        NamedCommands.registerCommand(
                "AlignHeadingToHub",
                new AlignHeadingToHub(
                        drive,
                        () -> -applyLeftDeadband(driver.getLeftY()),
                        () -> -applyLeftDeadband(driver.getLeftX()),
                        true));
        NamedCommands.registerCommand("IndexFlow", indexer.indexFlow());

        autoAlignHeadingToHub = new AlignHeadingToHub(drive, () -> 0, () -> 0, false);
        auto_autoAimShooter = new AutoAimShooter(drive, vision, shooter);

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

        autoChooser.addOption(
                "BBAll",
                new SequentialCommandGroup(
                        autoAlignHeadingToHub.until(autoAlignHeadingToHub.isLookingAtHub()),

                        new WaitUntilCommand(() -> auto_autoAimShooter.isAtTargetRpm())
                                .deadlineFor(makeAutoAimShooter()),

                        makeAutoAimShooter().withTimeout(2.0),

                        new ParallelDeadlineGroup(
                                new WaitCommand(5.0),
                                makeAutoAimShooter(),
                                indexer.indexFlow())));

        // Configure the button bindings
        configureButtonBindings();
    }

    private Command makeAutoAimShooter() {
        return new AutoAimShooter(drive, vision, shooter);
    }

    /**
     * Apply deadband to left joystick input (0.2 deadband). Values within the
     * deadband are set to 0,
     * values outside are scaled from 0 to 1.
     */
    private double applyLeftDeadband(double value) {
        if (Math.abs(value) < LEFT_JOYSTICK_DEADBAND) {
            return 0.0;
        }
        return Math.copySign(
                (Math.abs(value) - LEFT_JOYSTICK_DEADBAND) / (1.0 - LEFT_JOYSTICK_DEADBAND), value);
    }

    /**
     * Apply deadband to right joystick input (0.5 deadband). Values within the
     * deadband are set to 0,
     * values outside are scaled from 0 to 1.
     */
    private double applyRightDeadband(double value) {
        if (Math.abs(value) < RIGHT_JOYSTICK_DEADBAND) {
            return 0.0;
        }
        return Math.copySign(
                (Math.abs(value) - RIGHT_JOYSTICK_DEADBAND) / (1.0 - RIGHT_JOYSTICK_DEADBAND), value);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver
        drive.setDefaultCommand(new HeadingDrive(
                drive,
                () -> applyLeftDeadband(driver.getLeftX()),
                () -> applyLeftDeadband(driver.getLeftY()),
                () -> applyRightDeadband(driver.getRightX()),
                () -> applyRightDeadband(driver.getRightY()),
                MaxSpeed,
                MaxAngularRate,
                90));

        driver.a()
                .onTrue(Commands.runOnce(
                        () -> {
                            drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                        }, drive).ignoringDisable(true));

        driver.rightBumper()
                .whileTrue(new AlignHeadingToHub(
                        drive,
                        () -> applyLeftDeadband(driver.getLeftX()),
                        () -> applyLeftDeadband(driver.getLeftY()),
                        true));

        driver.rightTrigger()
                .whileTrue(new AlignHeadingToHub(
                        drive,
                        () -> applyLeftDeadband(driver.getLeftX()),
                        () -> applyLeftDeadband(driver.getLeftY()),
                        true).alongWith(
                                new AutoAimShooter(drive,
                                        vision,
                                        shooter)));
        // Operator
        operator.rightTrigger()
                .whileTrue(shooter.shoot(2800))
                .whileTrue(shooter.shroud(11));

        operator.rightBumper()
                .whileTrue(indexer.feed());

        operator.y()
                .whileTrue(shooter.shoot(6000));
        operator.x()
                .whileTrue(shooter.shoot(4000));

        operator.b()
                .whileTrue(indexer.index().alongWith(intake.intake()));

        operator.a()
                .whileTrue(shooter.shootPrecent(1));

        operator.leftStick()
                .toggleOnTrue(
                    new MidPassing(shooter)
                );
        
        operator.rightStick()
                .toggleOnTrue(
                    new AcrossPassing(shooter)
                );

        operator.leftBumper()
                .whileTrue(indexer.ohShit().alongWith(intake.out()));

        operator.leftTrigger()
                .whileTrue(indexer.indexFlow().alongWith(intake.intake()));

        operator.povUp()
                .onTrue(shooter.stepShroud(5));
        operator.povDown()
                .onTrue(shooter.stepShroud(-5));

        operator.povLeft()
                .onTrue(shooter.shroud(0));
        operator.povRight()
                .onTrue(shooter.shroud(13));

        operator.start()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    testShooterRPM = Math.min(testShooterRPM + RPM_INCREMENT, RPM_MAX);
                                    CommandScheduler.getInstance().schedule(shooter.shoot(testShooterRPM));
                                }));

        operator.back()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    testShooterRPM = Math.max(testShooterRPM - RPM_INCREMENT, RPM_MIN);
                                    CommandScheduler.getInstance().schedule(shooter.shoot(testShooterRPM));
                                }));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
