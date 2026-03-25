/** logging utility for slower logging rates 1626 - falcon robotics | Metuchen, New Jersey */
package frc.robot.util;

public class ThrottleLog {
  private int counter = 0;
  private int interval;

  public ThrottleLog(int interval) {
    this.interval = interval;
  }

  public ThrottleLog withInterval(int interval) {
    this.interval = interval;
    return this;
  }

  public void log(Runnable logAction) {
    counter++;
    if (counter % interval == 0) {
      logAction.run();
    }
  }
}
