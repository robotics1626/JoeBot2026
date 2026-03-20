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

/**
 * angelos - Drive train has constants
 *
 * @see frc.robot.generated.TunerConstants
 */
// class DriveConstants {
//     MAX_SPEED_MPS = 4.5;
//     MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * PI;

//     TRACK_WIDTH = 0.55;   // meters, left-right wheel distance
//     WHEEL_BASE  = 0.55;   // meters, front-back wheel distance

//     FRONT_LEFT_LOCATION  = new Translation2d(+WHEEL_BASE/2, +TRACK_WIDTH/2);
//     FRONT_RIGHT_LOCATION = new Translation2d(+WHEEL_BASE/2, -TRACK_WIDTH/2);
//     BACK_LEFT_LOCATION   = new Translation2d(-WHEEL_BASE/2, +TRACK_WIDTH/2);
//     BACK_RIGHT_LOCATION  = new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2);

//     KINEMATICS = new SwerveDriveKinematics(
//         FRONT_LEFT_LOCATION,
//         FRONT_RIGHT_LOCATION,
//         BACK_LEFT_LOCATION,
//         BACK_RIGHT_LOCATION
//     );

//     // CAN IDs
//     FL_DRIVE_ID = 5;
//     FL_TURN_ID = 6;
//     FL_ABS_ENCODER_ID = 14;

//     FR_DRIVE_ID = 7;
//     FR_TURN_ID = 8;
//     FR_ABS_ENCODER_ID = 15;

//     BL_DRIVE_ID = 1;
//     BL_TURN_ID = 2;
//     BL_ABS_ENCODER_ID = 11;

//     BR_DRIVE_ID = 3;
//     BR_TURN_ID = 4;
//     BR_ABS_ENCODER_ID = 12;

//     // Absolute encoder zero offsets in radians
//     FL_OFFSET = ...
//     FR_OFFSET = ...
//     BL_OFFSET = ...
//     BR_OFFSET = ...
// }
