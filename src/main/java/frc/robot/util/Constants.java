// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * 
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(25);


        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static double MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
        public static double DYNAMIC_MAX_ANGULAR_SPEED = 3 * Math.PI; // radians per second
        public static final double MIN_ANGULAR_SPEED = 4 * Math.PI; // radians per second
        // NEEDS TO BE TESTED vvv
        public static final double MAX_ANGULAR_SPEED = 8 * Math.PI; // radians per second

        public static final double MAX_TELEOP_SPEED_METERS_PER_SECOND = Units.feetToMeters(14.61);

        public static final double DIRECTION_SLEW_RATE = 6.28; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 80.0; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 80.0; // percent per second (1 = 100%)

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                // Front Positive, Left Positive
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Rear Right

        // Angular offsets of the modules relative to the chassis in radians
        // add 90 degrees to change the X and Y axis
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180 + 90);
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90 + 90);
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90 + 90);
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0 + 90);

        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        public static final boolean GYRO_REVERSED = Robot.isReal();
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_P = Robot.isSimulation() ? 0.5 : 1;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;

        public static ArrayList<CANSparkMax> motorList = new ArrayList<>();
    }

    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class AutoConstants {

        public static final double MAX_SPEED_METERS_PER_SECOND = 3; // 2.5 has worked very well for us so far
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.5; // 2.5; (2.5vel and 2.5accel output
                                                                                     // 3s runtime on _1_A)
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 9;// Math.PI/1.7;

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        public static final double XY_CORRECTION_P = 2.5;// 7;
        public static final double XY_CORRECTION_I = 0;
        public static final double XY_CORRECTION_D = 0.2;

        public static final double Y_CORRECTION_P = 2.5;// 6.03;
        public static final double Y_CORRECTION_I = 0;
        public static final double Y_CORRECTION_D = 0.2;

        public static final double ROTATION_CORRECTION_P = .63;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0.0025;

        public static final PIDConstants XY_PID_CONSTANTS = new PIDConstants(
                XY_CORRECTION_P, XY_CORRECTION_I, XY_CORRECTION_D);

        public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(
                ROTATION_CORRECTION_P, ROTATION_CORRECTION_I, ROTATION_CORRECTION_D);

        // Constraint for the motion-profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final HolonomicDriveController HDC = new HolonomicDriveController(
                new PIDController(
                        AutoConstants.XY_CORRECTION_P,
                        AutoConstants.XY_CORRECTION_I,
                        AutoConstants.XY_CORRECTION_D),
                new PIDController(
                        AutoConstants.XY_CORRECTION_P,
                        AutoConstants.XY_CORRECTION_I,
                        AutoConstants.XY_CORRECTION_D),
                new ProfiledPIDController(
                        AutoConstants.ROTATION_CORRECTION_P,
                        AutoConstants.ROTATION_CORRECTION_I,
                        AutoConstants.ROTATION_CORRECTION_D,
                        new TrapezoidProfile.Constraints(
                                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)));
    }

    public static final class FieldConstants {

        public static boolean IS_SIMULATION = Robot.isSimulation();

        public static final double ALIGNMENT_SPEED = 3;
        public static final double SNAP_TO_ANGLE_P = 0.0025;

        public static final double CONE_OFFSET_METERS = 0.542615;
        public static final double GRID_BARRIER_METERS = Units.inchesToMeters(12); // real is 14-15
        public static final double SUBSTATION_OFFSET_METERS = 0.7;
        public static final double ALLOWABLE_ERROR_METERS = Units.inchesToMeters(2);
        public static final double FIELD_WIDTH_METERS = 16.53;

        public static final double CHARGE_PAD_CORRECTION_P = 0.05;

        public static Alliance ALLIANCE = Alliance.Invalid;

        public static enum GameMode {
            DISABLED,
            AUTONOMOUS,
            TELEOP,
            TEST
        };

        public static GameMode GAME_MODE;
    }
}
