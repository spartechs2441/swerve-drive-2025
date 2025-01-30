// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.Constants.Controls
import frc.robot.Constants.OIConstants
import frc.robot.commands.LockRotation
import frc.robot.commands.TareCmd
import frc.robot.subsystems.DriveSubsystem

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems
    private val m_robotDrive = DriveSubsystem()

    val gyro: Double
        get() = m_robotDrive.gyroRotation().degrees

    // The driver's controller
    var m_driverController: XboxController = XboxController(OIConstants.kDriverControllerPort)

    private val autoChooser: SendableChooser<Command>

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        // Configure the button bindings
        configureButtonBindings()

        // Configure default commands
        m_robotDrive.defaultCommand = RunCommand(
            {
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.leftY, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.leftX, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.rightX, OIConstants.kDriveDeadband),
                    true
                )
            },
            m_robotDrive
        )


        println("=== Chooser ===")
        autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", autoChooser)
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a [edu.wpi.first.wpilibj.GenericHID] or one of its
     * subclasses ([ ] or [XboxController]), and then calling
     * passing it to a
     * [JoystickButton].
     */
    private fun configureButtonBindings() {
        JoystickButton(m_driverController, PS4Controller.Button.kR1.value)
            .whileTrue(
                RunCommand(
                    { m_robotDrive.setX() },
                    m_robotDrive
                )
            )
        JoystickButton(m_driverController, Controls.lockNorth).whileTrue(
            LockRotation(m_robotDrive, Rotation2d.fromDegrees(0.0))
        )
        JoystickButton(m_driverController, Controls.lockSouth).whileTrue(
            LockRotation(m_robotDrive, Rotation2d.fromDegrees(180.0))
        )
        JoystickButton(m_driverController, Controls.lockEast).whileTrue(
            LockRotation(m_robotDrive, Rotation2d.fromDegrees(270.0))
        )
        JoystickButton(m_driverController, Controls.lockWest).whileTrue(
            LockRotation(m_robotDrive, Rotation2d.fromDegrees(90.0))
        )
        JoystickButton(m_driverController, Controls.tare).whileTrue(
            TareCmd(m_robotDrive)
        )
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.selected
}
