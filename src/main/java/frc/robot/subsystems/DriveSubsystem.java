// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);
    private final MAXSwerveModule frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);
    private final MAXSwerveModule rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);
    private final MAXSwerveModule rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    // The gyro sensor
    private final Pigeon2 gyro = new Pigeon2(0);
    private double tare = 45;
    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getGyro(),
            new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
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
        var moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(moduleStates[0]);
        frontLeft.setDesiredState(moduleStates[1]);
        rearRight.setDesiredState(moduleStates[2]);
        rearLeft.setDesiredState(moduleStates[3]);
        // drive(0.5, 0, 0, false);
    }

    private ChassisSpeeds getRobotRelativeSpeeds(double xSpeedDelivered, double ySpeedDelivered, double rotDelivered, boolean fieldRelative) {
        return fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getGyro())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
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
        odometry.update(
                getGyro(),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */

    public Pose2d getPose() {
        var pose = odometry.getPoseMeters();
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
        odometry.resetPosition(
                getGyro(),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
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

        SmartDashboard.putNumber("FrontLeft Actual", frontLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("FrontRight Actual", frontRight.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("BackLeft Actual", rearLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("BackRight Actual", rearRight.getPosition().angle.getDegrees());

        SmartDashboard.putNumber("FrontLeft Speed", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FrontRight Speed", swerveModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BackLeft Speed", swerveModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BackRight Speed", swerveModuleStates[3].speedMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void formX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
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
