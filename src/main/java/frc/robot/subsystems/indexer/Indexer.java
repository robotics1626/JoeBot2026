package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO mIndexer;

  public Indexer(IndexerIO indexer) {
    this.mIndexer = indexer;
  }

  public Command index() {
    return startEnd(
        () -> {
          mIndexer.setSpeed(45);
          mIndexer.setFeeder(-20);
        },
        () -> {
          mIndexer.setSpeed(0);
          mIndexer.setFeeder(0);
        });
  }

  public Command justIndexer() {
    return startEnd(() -> mIndexer.setSpeed(45), () -> mIndexer.setSpeed(0));
  }

  public Command justIndexerRunOnce() {
    return runOnce(() -> mIndexer.setSpeed(45));
  }

  public Command stopJustIndexerRunOnce() {
    return runOnce(() -> mIndexer.setSpeed(0));
  }

  public Command feed() {
    return startEnd(() -> mIndexer.setFeeder(20), () -> mIndexer.setFeeder(0));
  }

  public Command feedRunOnce() {
    return runOnce(() -> mIndexer.setFeeder(20));
  }

  public Command stopFeedRunOnce() {
    return runOnce(() -> mIndexer.setFeeder(0));
  }

  public Command ohShit() {
    return startEnd(() -> mIndexer.setSpeed(-45), () -> mIndexer.setSpeed(0));
  }
}
