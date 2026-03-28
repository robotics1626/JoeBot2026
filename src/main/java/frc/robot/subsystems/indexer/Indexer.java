package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO mIndexer;
  private double feederTargetRPM = 0.0;

  public Indexer(IndexerIO indexer) {
    this.mIndexer = indexer;
  }

  public Command index() {
    return startEnd(
        () -> {
          mIndexer.setSpeed(.40);
          feederTargetRPM = -2000;
          mIndexer.setFeederRPM(feederTargetRPM);
        },
        () -> {
          mIndexer.setSpeed(0);
          feederTargetRPM = 0;
          mIndexer.setFeeder(0);
        });
  }

  public Command indexFlow() {
    return startEnd(
        () -> {
          mIndexer.setSpeed(.40);
          feederTargetRPM = 2000;
          mIndexer.setFeederRPM(feederTargetRPM);
        },
        () -> {
          mIndexer.setSpeed(0);
          feederTargetRPM = 0;
          mIndexer.setFeeder(0);
        });
  }

  public Command justIndexer() {
    return startEnd(() -> mIndexer.setSpeed(.40), () -> mIndexer.setSpeed(0));
  }

  public Command justIndexerRunOnce() {
    return runOnce(() -> mIndexer.setSpeed(.40));
  }

  public Command stopJustIndexerRunOnce() {
    return runOnce(() -> mIndexer.setSpeed(0));
  }

  public Command feed() {
    return startEnd(
        () -> {
          feederTargetRPM = 2000;
          mIndexer.setSpeed(.40);
          mIndexer.setFeederRPM(feederTargetRPM);
        },
        () -> {
          feederTargetRPM = 0;
          mIndexer.setSpeed(0);
          mIndexer.setFeeder(0);
        });
  }

  public Command feedRunOnce() {
    return runOnce(
        () -> {
          feederTargetRPM = 2000;
          mIndexer.setFeederRPM(feederTargetRPM);
        });
  }

  public Command stopFeedRunOnce() {
    return runOnce(
        () -> {
          feederTargetRPM = 0;
          mIndexer.setFeeder(0);
        });
  }

  public Command ohShit() {
    return startEnd(() -> mIndexer.setSpeed(-.40), () -> mIndexer.setSpeed(0));
  }

  public BooleanSupplier untilTimer(Time time) {
    // does a loop in however much time passed into the arguments. If the time is up, return true,
    // else false.
    long startTime = System.currentTimeMillis();
    return () -> (System.currentTimeMillis() - startTime) >= time.in(Milliseconds);
  }

  @Override
  public void periodic() {
    // Log feeder target and actual RPM (throttled to 50Hz for lower bandwidth)
    Logger.recordOutput("Indexer/Feeder/TargetRPM", feederTargetRPM);
    Logger.recordOutput("Indexer/Feeder/ActualRPM", mIndexer.getFeeder());
  }
}
