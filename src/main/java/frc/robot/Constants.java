// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 1.0; // 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = 0;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = 0;
        public static final double kBackRightChassisAngularOffset = 0;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 41;
        public static final int kFrontRightDrivingCanId = 42;
        public static final int kRearLeftDrivingCanId = 43;
        public static final int kRearRightDrivingCanId = 44;

        public static final int kFrontLeftTurningCanId = 51;
        public static final int kFrontRightTurningCanId = 52;
        public static final int kRearLeftTurningCanId = 53;
        public static final int kRearRightTurningCanId = 54;

        public static final boolean kGyroReversed = false;
    }

    public static final class ElevatorConstants {
        public static final int canId = 12;
        public static final int limitSwitchDIo = 0;
        // Test this when changing the elevator
        public static final int encoderLimit = 387;
        public static final int voltage = 3;

    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class Controls {
        public static final int lockNorth = XboxController.Button.kY.value;
        public static final int lockEast = XboxController.Button.kB.value;
        public static final int lockWest = XboxController.Button.kX.value;
        public static final int lockSouth = XboxController.Button.kA.value;
        public static final int lightTrack = XboxController.Button.kRightBumper.value;
        public static final int tare = XboxController.Button.kStart.value;
        public static final int pistonOut = XboxController.Button.kA.value;
        public static final int pistonIn = XboxController.Button.kB.value;

        //Controller Controls
        public static final int yMovement = XboxController.Axis.kLeftX.value; //The X and Y Movement are switched
        public static final int xMovement = XboxController.Axis.kLeftY.value;
        public static final int rotation = XboxController.Axis.kRightX.value;
    }
}
