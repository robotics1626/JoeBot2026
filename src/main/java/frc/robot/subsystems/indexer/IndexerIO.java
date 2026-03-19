package frc.robot.subsystems.indexer;

public interface IndexerIO {
  /**
   * Set speed of the main indexer in precentage
   *
   * @param speed
   */
  public void setSpeed(double speed);

  /**
   * Get current speed of main indexer
   *
   * @return
   */
  public double getSpeed();

  /**
   * Set the speed of the feeder in precentage
   *
   * @param speed
   */
  public void setFeeder(double speed);

  /**
   * Get speed of the feeder.
   *
   * @return
   */
  public double getFeeder();
}
