package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class IndexerIOTalon implements IndexerIO {
  private final TalonFX mIndexer;
  private final TalonFX mFeeder;

  public IndexerIOTalon() {
    mIndexer = new TalonFX(IndexerConstants.Motors.kIndexer, Constants.K_CAN_BUS);
    mFeeder = new TalonFX(IndexerConstants.Motors.kFeeder, Constants.K_CAN_BUS);

    var BaseIndexerConfigs = new TalonFXConfiguration();
    BaseIndexerConfigs.MotorOutput = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    BaseIndexerConfigs.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Current.ofRelativeUnits(35, Amps))
        .withSupplyCurrentLimitEnable(true);

    mIndexer.getConfigurator().apply(BaseIndexerConfigs);

    var BaseFeederConfigs = new TalonFXConfiguration();
    BaseFeederConfigs.MotorOutput = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    BaseFeederConfigs.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Current.ofRelativeUnits(35, Amps))
        .withSupplyCurrentLimitEnable(true);

    mFeeder.getConfigurator().apply(BaseFeederConfigs);
  }

  @Override
  public void setSpeed(double speed) {
    mIndexer.set(speed);
  }

  @Override
  public double getSpeed() {
    return RotationsPerSecond.of(mIndexer.getVelocity().getValueAsDouble()).in(RPM);
  }

  @Override
  public void setFeeder(double speed) {
    mFeeder.set(speed);
  }

  @Override
  public double getFeeder() {
    return RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(RPM);
  }
}
