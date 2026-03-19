package frc.robot.subsystems.shooter;

/**
 * Interface for the shooter
 */
public interface ShooterIO {

    // Main shooting mechanism (flywheel)

    /**
     * Set the speed of the shooter in precentage.
     * @param precent
     */
    public void setSpeed(double precent);

    /**
     * Set the speed of the shooter in RPM.
     * @apiNote Converted from RPM->RPS.
     * @param RPM
     */
    public void setSpeedRPM(double RPM);

    /**
     * Get current speed of the shooter in RPM.
     * @return RPM.
     */
    public double getSpeedRPM();
    
    // Shroud control

    /**
     * Set the shroud to certain degrees.
     * @param degrees
     */
    public void setShroud(double degrees);

    /**
     * Get the shrouds current position as degrees.
     * @return General degrees
     */
    public double getShroud();
}
