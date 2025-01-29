// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

import java.security.Provider;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);
    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);
    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    // The gyro sensor
    private final Pigeon2 gyro = new Pigeon2(0);
    private double tare = 0;
    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getGyro(),
            new SwerveModulePosition[]{
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        // All other subsystem initialization
        // ...

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            // This exists because intellij gives me the "cannot resolve x"
            // without giving me the actual place it errors, this is to narrow down errors
            Supplier<Pose2d> getPose = this::getPose;
            Consumer<Pose2d> resetPose = this::resetPose;
            Supplier<ChassisSpeeds> getRobotRelativeSpeeds = this::getRobotRelativeSpeeds;
            Consumer<ChassisSpeeds> robotDriveRelative = this::robotDriveRelative;

            // Configure AutoBuilder
            AutoBuilder.configure(
                    getPose,
                    resetPose,
                    getRobotRelativeSpeeds,
                    robotDriveRelative, // (speeds, feedforwards) -> driveRobotRelative((Object) speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
                    ),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            );
            System.out.println("=== Done configuring ===");
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace(); // DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.printStackTrace());
        }
    }


    public void robotDriveRelative(ChassisSpeeds speeds) {
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        drive(0.5, 0, 0, false);
    }

    private ChassisSpeeds getRobotRelativeSpeeds(double xSpeedDelivered, double ySpeedDelivered, double rotDelivered, boolean fieldRelative) {
        return fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getGyro())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
    }

    public Rotation2d getGyro() {
        double angle = this.gyro.getYaw().getValueAsDouble();
        return Rotation2d.fromDegrees(angle - tare);
    }

    public void tare() {
        this.tare += getGyro().getDegrees();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                getGyro(),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */

    public Pose2d getPose() {
        var pose = m_odometry.getPoseMeters();
        if (pose == null) {
            throw new RuntimeException("WHY ARE YOU NULL");
        }
        return pose;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(
                getGyro(),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var chassisSpeed = getRobotRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                chassisSpeed
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);


        SmartDashboard.putNumber("FrontLeft Desired", swerveModuleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("FrontRight Desired", swerveModuleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("BackLeft Desired", swerveModuleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("BackRight Desired", swerveModuleStates[3].angle.getDegrees());

        SmartDashboard.putNumber("FrontLeft Actual", m_frontLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("FrontRight Actual", m_frontRight.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("BackLeft Actual", m_rearLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("BackRight Actual", m_rearRight.getPosition().angle.getDegrees());

        SmartDashboard.putNumber("FrontLeft Speed", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FrontRight Speed", swerveModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BackLeft Speed", swerveModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BackRight Speed", swerveModuleStates[3].speedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        this.gyro.reset();
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
