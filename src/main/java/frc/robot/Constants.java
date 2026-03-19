// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final CANBus K_CAN_BUS = CANBus.roboRIO();
}

class DriveConstants {
    MAX_SPEED_MPS = 4.5;
    MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * PI;

    TRACK_WIDTH = 0.55;   // meters, left-right wheel distance
    WHEEL_BASE  = 0.55;   // meters, front-back wheel distance

    FRONT_LEFT_LOCATION  = new Translation2d(+WHEEL_BASE/2, +TRACK_WIDTH/2);
    FRONT_RIGHT_LOCATION = new Translation2d(+WHEEL_BASE/2, -TRACK_WIDTH/2);
    BACK_LEFT_LOCATION   = new Translation2d(-WHEEL_BASE/2, +TRACK_WIDTH/2);
    BACK_RIGHT_LOCATION  = new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2);

    KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_LOCATION,
        FRONT_RIGHT_LOCATION,
        BACK_LEFT_LOCATION,
        BACK_RIGHT_LOCATION
    );

    // CAN IDs
    FL_DRIVE_ID = ...
    FL_TURN_ID = ...
    FL_ABS_ENCODER_ID = ...

    FR_DRIVE_ID = ...
    FR_TURN_ID = ...
    FR_ABS_ENCODER_ID = ...

    BL_DRIVE_ID = ...
    BL_TURN_ID = ...
    BL_ABS_ENCODER_ID = ...

    BR_DRIVE_ID = ...
    BR_TURN_ID = ...
    BR_ABS_ENCODER_ID = ...

    // Absolute encoder zero offsets in radians
    FL_OFFSET = ...
    FR_OFFSET = ...
    BL_OFFSET = ...
    BR_OFFSET = ...
}
