// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoLimelightCmd;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.LockRotation;
import frc.robot.commands.TareCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LimelightSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive;
    public LimelightSubsystem m_limelight;

    public double getGyro() {
        return m_robotDrive.getGyro().getDegrees();
    }

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

//    private final SendableChooser<Command> autoChooser;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_robotDrive = new DriveSubsystem();
        var networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_limelight = new LimelightSubsystem(networkTable);
        // Configure the button bindings
        configureButtonBindings();

        NamedCommands.registerCommand("Limelight", new AutoLimelightCmd(m_robotDrive, m_limelight, 0.5));

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));


        System.out.println("=== Chooser ===");
//        autoChooser = AutoBuilder.buildAutoChooser();
//        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

//        new JoystickButton(m_driverController, Constants.Controls.lockNorth).whileTrue(
//                new LockRotation(m_robotDrive, Rotation2d.fromDegrees(0))
//        );
//        new JoystickButton(m_driverController, Constants.Controls.lockSouth).whileTrue(
//                new LockRotation(m_robotDrive, Rotation2d.fromDegrees(180))
//        );
//        new JoystickButton(m_driverController, Constants.Controls.lockEast).whileTrue(
//                new LockRotation(m_robotDrive, Rotation2d.fromDegrees(270))
//        );
//        new JoystickButton(m_driverController, Constants.Controls.lockWest).whileTrue(
//                new LockRotation(m_robotDrive, Rotation2d.fromDegrees(90))
//        );
          new JoystickButton(m_driverController, Constants.Controls.lightTrack).whileTrue(
                  new LimelightCmd(m_limelight, m_robotDrive, m_driverController)
          );
//        new JoystickButton(m_driverController, Constants.Controls.tare).whileTrue(
//                new TareCmd(m_robotDrive)
//        );

    }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
    public Command getAutonomousCommand() {
//        return autoChooser.getSelected().andThen(new Command() {
//            @Override
//            public void initialize() {
//                System.out.println("Hello end of auto");
//            }
//        });
        return null;
    }
}
