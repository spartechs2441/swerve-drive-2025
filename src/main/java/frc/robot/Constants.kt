// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.XboxController

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        const val kMaxSpeedMetersPerSecond: Double = 1.0 // 4.8;
        const val kMaxAngularSpeed: Double = 2 * Math.PI // radians per second

        // Chassis configuration
        val kTrackWidth: Double = Units.inchesToMeters(22.5)

        // Distance between centers of right and left wheels on robot
        val kWheelBase: Double = Units.inchesToMeters(22.5)

        // Distance between front and back wheels on robot
        val kDriveKinematics: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        )

        // Angular offsets of the modules relative to the chassis in radians
        const val kFrontLeftChassisAngularOffset: Double = 0.0
        const val kFrontRightChassisAngularOffset: Double = 0.0
        const val kBackLeftChassisAngularOffset: Double = 0.0
        const val kBackRightChassisAngularOffset: Double = 0.0

        // SPARK MAX CAN IDs
        const val kFrontLeftDrivingCanId: Int = 41
        const val kFrontRightDrivingCanId: Int = 42
        const val kRearLeftDrivingCanId: Int = 43
        const val kRearRightDrivingCanId: Int = 44

        const val kFrontLeftTurningCanId: Int = 51
        const val kFrontRightTurningCanId: Int = 52
        const val kRearLeftTurningCanId: Int = 53
        const val kRearRightTurningCanId: Int = 54

        const val kGyroReversed: Boolean = false
    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        const val kDrivingMotorPinionTeeth: Int = 14

        // Calculations required for driving motor conversion factors and feed forward
        const val kDrivingMotorFreeSpeedRps: Double = NeoMotorConstants.kFreeSpeedRpm / 60
        const val kWheelDiameterMeters: Double = 0.0762
        const val kWheelCircumferenceMeters: Double = kWheelDiameterMeters * Math.PI

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        const val kDrivingMotorReduction: Double = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
        const val kDriveWheelFreeSpeedRps: Double = ((kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction)
    }

    object OIConstants {
        const val kDriverControllerPort: Int = 0
        const val kDriveDeadband: Double = 0.05
    }

    object AutoConstants {
        const val kMaxSpeedMetersPerSecond: Double = 3.0
        const val kMaxAccelerationMetersPerSecondSquared: Double = 3.0
        const val kMaxAngularSpeedRadiansPerSecond: Double = Math.PI
        const val kMaxAngularSpeedRadiansPerSecondSquared: Double = Math.PI

        const val kPXController: Double = 1.0
        const val kPYController: Double = 1.0
        const val kPThetaController: Double = 1.0

        // Constraint for the motion profiled robot angle controller
        val kThetaControllerConstraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        )
    }

    object NeoMotorConstants {
        const val kFreeSpeedRpm: Double = 5676.0
    }

    object Controls {
        val lockNorth: Int = XboxController.Button.kY.value
        val lockEast: Int = XboxController.Button.kB.value
        val lockWest: Int = XboxController.Button.kX.value
        val lockSouth: Int = XboxController.Button.kA.value
        val lightTrack: Int = XboxController.Button.kRightBumper.value
        val tare: Int = XboxController.Button.kStart.value

        //Controller Controls
        val yMovement: Int = XboxController.Axis.kLeftX.value //The X and Y Movement are switched
        val xMovement: Int = XboxController.Axis.kLeftY.value
        val rotation: Int = XboxController.Axis.kRightX.value
    }
}
