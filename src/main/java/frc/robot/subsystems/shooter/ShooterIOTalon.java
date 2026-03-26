package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterIOTalon implements ShooterIO {
  private final TalonFX mShooterLeader;
  private final TalonFX mShooterFollower;

  private final TalonFX mShroudController;

  public ShooterIOTalon() {
    mShooterLeader = new TalonFX(ShooterConstants.Motors.kShooterLeader, Constants.K_CAN_BUS);
    mShooterFollower = new TalonFX(ShooterConstants.Motors.kShooterFollower, Constants.K_CAN_BUS);
    mShroudController = new TalonFX(ShooterConstants.Motors.kShroud, Constants.K_CAN_BUS);

    // Shooter configurations

    var BaseShooterConfigs = new TalonFXConfiguration();
    BaseShooterConfigs.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.PID.kShooterOldP)
            .withKI(ShooterConstants.PID.kShooterOldI)
            .withKD(ShooterConstants.PID.kShooterOldD)
            .withKS(ShooterConstants.PID.kShooterS)
            .withKV(ShooterConstants.PID.kShooterV);

    BaseShooterConfigs.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Current.ofRelativeUnits(30, Amps))
        .withSupplyCurrentLimitEnable(true);

    BaseShooterConfigs.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    var shooterLeaderConfigurator = mShooterLeader.getConfigurator();
    shooterLeaderConfigurator.apply(BaseShooterConfigs);

    var shooterFollowerConfigurator = mShooterFollower.getConfigurator();
    shooterFollowerConfigurator.apply(
        BaseShooterConfigs.withMotorOutput(
            BaseShooterConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)));

    // Shroud configurations

    var BaseShroudConfigs = new TalonFXConfiguration();
    BaseShroudConfigs.Slot0 = new Slot0Configs()
        .withKP(ShooterConstants.PID.kShroudP)
        .withKI(ShooterConstants.PID.kShroudI)
        .withKD(ShooterConstants.PID.kShooterD);

    BaseShroudConfigs.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Current.ofRelativeUnits(30, Amps))
        .withSupplyCurrentLimitEnable(true);

    BaseShroudConfigs.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    var shroudConfigurator = mShroudController.getConfigurator();
    shroudConfigurator.apply(BaseShroudConfigs);
  }

  @Override
  public void setSpeed(double precent) {
    mShooterLeader.set(precent);
    mShooterFollower.setControl(
        new Follower(
            mShooterLeader.getDeviceID(),
            MotorAlignmentValue.Opposed)); // ag - is there another method where i dont have to set control?
  }

  @Override
  public void setSpeedRPM(double rpm) {
    var request = new VelocityVoltage(AngularVelocity.ofRelativeUnits(rpm, RPM));
    double rps = AngularVelocity.ofRelativeUnits(rpm, RPM).in(RotationsPerSecond);
    double feedForwardVolts = ShooterConstants.PID.kShooterV * rps;
    mShooterLeader.setControl(request.withSlot(0).withFeedForward(feedForwardVolts));
    mShooterFollower.setControl(
        new Follower(mShooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    // Log control values for PID/VS tuning
    try {
      double measuredRpm = getLeaderRPM();
      double errorRpm = rpm - measuredRpm;
      double appliedVolts = Double.NaN;
      try {
        appliedVolts = mShooterLeader.getMotorVoltage().getValueAsDouble();
      } catch (Exception ignored) {
      }

      Logger.recordOutput("Shooter/Control/TargetRPM", rpm);
      Logger.recordOutput("Shooter/Control/FeedForwardVolts", feedForwardVolts);
      Logger.recordOutput("Shooter/Control/KS", ShooterConstants.PID.kShooterS);
      Logger.recordOutput("Shooter/Control/KV", ShooterConstants.PID.kShooterV);
      Logger.recordOutput("Shooter/Control/KP", ShooterConstants.PID.kShooterP);
      Logger.recordOutput("Shooter/Control/KI", ShooterConstants.PID.kShooterI);
      Logger.recordOutput("Shooter/Control/KD", ShooterConstants.PID.kShooterD);
      Logger.recordOutput("Shooter/Measured/LeaderRPM", measuredRpm);
      Logger.recordOutput("Shooter/Control/ErrorRPM", errorRpm);
      Logger.recordOutput("Shooter/Measured/LeaderAppliedVolts", appliedVolts);
    } catch (Exception e) {
      // Don't let logging interfere with control
    }
  }

  @Override
  public double getLeaderRPM() {
    return RotationsPerSecond.of(mShooterLeader.getVelocity().getValueAsDouble()).in(RPM);
  }

  @Override
  public double getFollowerRPM() {
    return RotationsPerSecond.of(mShooterFollower.getVelocity().getValueAsDouble()).in(RPM);
  }

  @Override
  public void setShroud(double degrees) {
    var calculatedRotations = (Angle.ofRelativeUnits(degrees, Degrees)).in(Rotations) * 157.5;
    var request = new PositionVoltage(Angle.ofRelativeUnits(calculatedRotations, Rotations));
    mShroudController.setControl(request.withSlot(0).withFeedForward(calculatedRotations * .13));
  }

  @Override
  public void moveShroud(double degrees) {
    var lastRecordedPos = mShroudController.getPosition().getValue().in(Rotations);
    mShroudController.setControl(
        new PositionVoltage((lastRecordedPos + Angle.ofBaseUnits(degrees, Degrees).in(Rotations))));
  }

  @Override
  public double getShroud() {
    return Rotations.of(mShroudController.getPosition().getValueAsDouble()).in(Degrees) / 157.5;
  }

  @Override
  public void zero() {
    mShooterLeader.set(0);
    mShooterFollower.setControl(
        new Follower(mShooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    mShroudController.setControl(new PositionVoltage(0).withSlot(0));
  }
}
