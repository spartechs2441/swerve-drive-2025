// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DriveConstants
import java.util.function.Consumer
import java.util.function.Supplier

class DriveSubsystem : SubsystemBase() {
    // Create MAXSwerveModules
    private val m_frontLeft = MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset
    )
    private val m_frontRight = MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset
    )
    private val m_rearLeft = MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset
    )
    private val m_rearRight = MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset
    )

    // The gyro sensor
    private val gyro = Pigeon2(0)
    private var tare = 0.0

    // Odometry class for tracking robot pose
    var m_odometry: SwerveDriveOdometry = SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        gyroRotation(),
        arrayOf(
            m_frontLeft.position,
            m_frontRight.position,
            m_rearLeft.position,
            m_rearRight.position
        )
    )

    /**
     * Creates a new DriveSubsystem.
     */
    init {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve)

        // All other subsystem initialization
        // ...

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try {
            val config = RobotConfig.fromGUISettings()

            // This exists because intellij gives me the "cannot resolve x"
            // without giving me the actual place it errors, this is to narrow down errors
            val getPose = Supplier { this.pose }
            val resetPose = Consumer { pose: Pose2d -> this.resetPose(pose) }
            val getRobotRelativeSpeeds = Supplier { this.robotRelativeSpeeds }
            val robotDriveRelative =
                Consumer { speeds: ChassisSpeeds? -> this.robotDriveRelative(speeds) }

            // Configure AutoBuilder
            AutoBuilder.configure(
                getPose,
                resetPose,
                getRobotRelativeSpeeds,
                robotDriveRelative,  // (speeds, feedforwards) -> driveRobotRelative((Object) speeds),
                PPHolonomicDriveController(
                    PIDConstants(5.0, 0.0, 0.0),
                    PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                {
                    val alliance = DriverStation.getAlliance()
                    if (alliance.isPresent) {
                        return@configure alliance.get() == Alliance.Red
                    }
                    false
                },
                this
            )
            println("=== Done configuring ===")
        } catch (e: Exception) {
            // Handle exception as needed
            e.printStackTrace() // DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.printStackTrace());
        }
    }


    fun robotDriveRelative(speeds: ChassisSpeeds?) {
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
        drive(0.5, 0.0, 0.0, false)
    }

    private fun getRobotRelativeSpeeds(
        xSpeedDelivered: Double,
        ySpeedDelivered: Double,
        rotDelivered: Double,
        fieldRelative: Boolean
    ): ChassisSpeeds {
        return if (fieldRelative)
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                gyroRotation()
            )
        else
            ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    }

    val robotRelativeSpeeds: ChassisSpeeds
        get() = DriveConstants.kDriveKinematics.toChassisSpeeds(
            arrayOf(m_frontLeft.state,
            m_frontRight.state,
            m_rearLeft.state,
            m_rearRight.state)
        )

    fun gyroRotation(): Rotation2d {
        val angle = gyro.yaw.valueAsDouble
        return Rotation2d.fromDegrees(angle - tare)
    }

    fun tare() {
        this.tare += gyroRotation().degrees
    }

    override fun periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
            gyroRotation(),
            arrayOf(
                m_frontLeft.position,
                m_frontRight.position,
                m_rearLeft.position,
                m_rearRight.position
            )
        )
    }

    val pose: Pose2d
        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        get() {
            val pose = m_odometry.poseMeters
            return pose
        }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    fun resetPose(pose: Pose2d) {
        m_odometry.resetPosition(
            gyroRotation(),
            arrayOf(
                m_frontLeft.position,
                m_frontRight.position,
                m_rearLeft.position,
                m_rearRight.position
            ),
            pose
        )
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     * field.
     */
    fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        // Convert the commanded speeds into the correct units for the drivetrain
        val xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        val ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        val rotDelivered = rot * DriveConstants.kMaxAngularSpeed

        val chassisSpeed = getRobotRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative)

        val swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            chassisSpeed
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )


        SmartDashboard.putNumber("FrontLeft Desired", swerveModuleStates[0].angle.degrees)
        SmartDashboard.putNumber("FrontRight Desired", swerveModuleStates[1].angle.degrees)
        SmartDashboard.putNumber("BackLeft Desired", swerveModuleStates[2].angle.degrees)
        SmartDashboard.putNumber("BackRight Desired", swerveModuleStates[3].angle.degrees)

        SmartDashboard.putNumber("FrontLeft Actual", m_frontLeft.position.angle.degrees)
        SmartDashboard.putNumber("FrontRight Actual", m_frontRight.position.angle.degrees)
        SmartDashboard.putNumber("BackLeft Actual", m_rearLeft.position.angle.degrees)
        SmartDashboard.putNumber("BackRight Actual", m_rearRight.position.angle.degrees)

        SmartDashboard.putNumber("FrontLeft Speed", swerveModuleStates[0].speedMetersPerSecond)
        SmartDashboard.putNumber("FrontRight Speed", swerveModuleStates[1].speedMetersPerSecond)
        SmartDashboard.putNumber("BackLeft Speed", swerveModuleStates[2].speedMetersPerSecond)
        SmartDashboard.putNumber("BackRight Speed", swerveModuleStates[3].speedMetersPerSecond)

        m_frontLeft.setDesiredState(swerveModuleStates[0])
        m_frontRight.setDesiredState(swerveModuleStates[1])
        m_rearLeft.setDesiredState(swerveModuleStates[2])
        m_rearRight.setDesiredState(swerveModuleStates[3])
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    fun setX() {
        m_frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        m_frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        m_rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        m_rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        m_frontLeft.setDesiredState(desiredStates[0])
        m_frontRight.setDesiredState(desiredStates[1])
        m_rearLeft.setDesiredState(desiredStates[2])
        m_rearRight.setDesiredState(desiredStates[3])
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    fun resetEncoders() {
        m_frontLeft.resetEncoders()
        m_rearLeft.resetEncoders()
        m_frontRight.resetEncoders()
        m_rearRight.resetEncoders()
    }

    /**
     * Zeroes the heading of the robot.
     */
    fun zeroHeading() {
        gyro.reset()
    }
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    //    public double getHeading() {
    //        return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
    //    }
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    //    public double getTurnRate() {
    //        return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    //    }
}
