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
   * Set the speed of the feeder in RPM
   *
   * @apiNote Converted from RPM->RPS.
   * @param RPM
   */
  public void setFeederRPM(double RPM);

  /**
   * Get speed of the feeder.
   *
   * @return
   */
  public double getFeeder();
}
