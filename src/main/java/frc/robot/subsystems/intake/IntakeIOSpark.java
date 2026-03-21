package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax intakeLeader;
  private final SparkMax intakeFollower;

  public IntakeIOSpark() {
    intakeLeader = new SparkMax(IntakeConstants.Motors.kLeader, MotorType.kBrushless);
    intakeFollower = new SparkMax(IntakeConstants.Motors.kFollower, MotorType.kBrushless);

    var leaderConfigs = new SparkMaxConfig();
    leaderConfigs.inverted(false);
    // leaderConfigs.smartCurrentLimit(30);

    intakeLeader.configure(
        leaderConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var followerConfigs = new SparkMaxConfig();
    followerConfigs.inverted(true);
    followerConfigs.follow(intakeLeader, true);
    // followerConfigs.smartCurrentLimit(30);
  }

  @Override
  public void run(double precent) {
    intakeLeader.set(precent);
  }
}
