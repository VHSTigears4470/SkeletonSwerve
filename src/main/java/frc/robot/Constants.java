// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants {
    public static final double WHEEL_DIAMETER = 4;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1/8.41;
    public static final double TURN_MOTOR_GEAR_RATIO = 1/12.8;
    public static final double DRIVE_ENCODER_ROTATION_TO_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double TURN_ENCODER_ROTATION_TO_RADIANS = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI;
    public static final double DRIVE_ENCODER_RPM_TO_METER_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METER / 60;
    public static final double TURN_ENCODER_RPM_TO_METER_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60;

    public static final double P_TURN = 0.018; // originally 0.05
    public static final double I_TURN = 0; // originally 0
    public static final double D_TURN = 0.2; // originally 0.05

    public static final double P_DRIVE = 0.0000000001; // originally 0.000000005
    public static final double I_DRIVE = 0; // originally 0
    public static final double D_DRIVE = 0; // originally 0.4
    public static final double FF_DRIVE = 0; // originally 0
    public static final double IZONE_DRIVE = 0; // originally 0
    public static final double MIN_DRIVE = -1; // originally -1
    public static final double MAX_DRIVE = 1; // originally -1
  }

  public static class DriveConstants {

    public static final double TRACK_WIDTH = Units.inchesToMeters(19); // Check
    // Distance between right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(19); // Check
    // Distance between front and back wheels
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
    );

    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 2; // Check
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 4; // Check
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 6; // Check
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 8; // Check

    public static final int BACK_LEFT_TURNING_MOTOR_PORT = 1; // Check
    public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 3; // Check
    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5; // Check
    public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 7; // Check

    public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = false; // Check
    public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = false; // Check
    public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = false; // Check
    public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = false; // Check

    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false; // Check
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true; // Check
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false; // Check
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = true; // Check

    public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 9; // Check
    public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 10; // Check
    public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 11; // Check
    public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 12; // Check

    public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false; // Check / Modify
    public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false; // Check / Modify
    public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false; // Check / Modify
    public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false; // Check / Modify

    public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS = 114.47890625; // Check / Modify
    public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS = 32.98046875; // Check / Modify
    public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS = 298.6015625; // Check / Modify
    public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS = 34.134765625; // Check / Modify

    public static final double PHYSICAL_MAX_SPEED_METER_PER_SECOND = 4; // Modify HELP
    public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND = 2 * 2 * Math.PI; // Modify HELP

    public static final double TELE_DRIVE_MAX_SPEED_METER_PER_SECOND = PHYSICAL_MAX_SPEED_METER_PER_SECOND * 0.25; // 25% power // Modify
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND * 0.25; // 25% power // Modify
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNIT_PER_SECOND = 3; // Modify
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNIT_PER_SECOND = 3; // Modify
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0; // Check

    public static final int DRIVER_Y_AXIS = 1; // Check
    public static final int DRIVER_X_AXIS = 0; // Check
    public static final int DRIVER_ROT_AXIS = 4; // Check
    public static final int DRIVER_FIELD_ORIENTED_BUTTON_IDX = 1; // Check

    public static final double DRIVER_DEADBAND = 0.05; // Modify
  }

  public static final class AutoConstants {
     public static final double MAX_SPEED_METER_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND * 0.25; // 25% power
        public static final double MAX_ANGULAR_SPEED_RADIAN_PER_SECOND = DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND * 0.1; // 10% power
        public static final double MAX_ACCELERATION_METER_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED = Math.PI / 4;
        public static final double P_X_CONTROLLER = 1.5; // Modify
        public static final double P_Y_CONTROLLER = 1.5; // Modify
        public static final double P_THETA_CONTROLLER = 3;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIAN_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED);
  }
}
