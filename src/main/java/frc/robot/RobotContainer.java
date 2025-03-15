// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.HashMap;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem robotDrive;
    private final ChuteSubsystem chuteSub;
    public final LimelightSubsystem limelight;
    private final ElevatorSubsystem eleSub;
    private final ConveyorSubsystem conveySub;
    private final IntakeSubsystem intakeSub;
    private final LEDManager ledManager;

    public double getGyro() {
        return robotDrive.getGyro().getDegrees();
    }

    // The driver's controller
    XboxController driverController;
    Joystick flightstickController;

    public void nyanCat() {
//        ledManager.updateLEDs(LEDStatus.RANDOM);
    }

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ledManager = null; // new LEDManager(new AddressableLED(Constants.LED.ledPort), Constants.LED.ledLength);
        robotDrive = new DriveSubsystem();
        chuteSub = new ChuteSubsystem();
        var networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelight = new LimelightSubsystem(networkTable);
        eleSub = new ElevatorSubsystem(chuteSub);
        conveySub = new ConveyorSubsystem();
        intakeSub = new IntakeSubsystem();

        driverController = new XboxController(OIConstants.kDriverControllerPort);
        flightstickController = new Joystick(OIConstants.kFlightstickControllerPort);


        // Configure the button bindings
        configureButtonBindings();

        NamedCommands.registerCommand("Limelight", new AutoLimelightCmd(robotDrive, limelight, 0.5));
        NamedCommands.registerCommand("ElevatorL2", new ElevatorMacroCmd(260, eleSub));
        NamedCommands.registerCommand("ElevatorL3", new ElevatorMacroCmd(Constants.ElevatorConstants.encoderLimit, eleSub));
        NamedCommands.registerCommand("ShootMacro", new ChuteMacroCmd(chuteSub, eleSub));

        // Configure default commands
        robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> robotDrive.drive(
                                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        robotDrive));

        intakeSub.setDefaultCommand(
                new IntakeCmd(intakeSub, driverController)
        );

        System.out.println("=== Chooser ===");
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        // The checker sees all
        CommandScheduler.getInstance().schedule(new ControllerCheckCmd());

        // XBOX Controls
        new JoystickButton(driverController, Constants.Controls.conveyUp)
                .onTrue(new ConveyorInCmd(conveySub))
                .onFalse(new ConveyorStopCmd(conveySub)
        );
        new JoystickButton(driverController, Constants.Controls.conveyDown)
                .onTrue(new ConveyorOutCmd(conveySub))
                .onFalse(new ConveyorStopCmd(conveySub)
        );
        new JoystickButton(driverController, Constants.Controls.aprilTagTrack).onTrue(
                new LimelightCmd(limelight, robotDrive, driverController)
        );
        new JoystickButton(driverController, Constants.Controls.hingeDown)
                .onTrue(new HingeDownCmd(intakeSub))
                .onFalse(new HingeStopCmd(intakeSub)
        );
        new JoystickButton(driverController, Constants.Controls.hingeUp)
                .onTrue(new HingeUpCmd(intakeSub))
                .onFalse(new HingeStopCmd(intakeSub)
        );

        // Flightstick Controls
        new JoystickButton(flightstickController, Constants.Controls.macroL2).onTrue(
                new ElevatorMacroCmd(250, eleSub)
        );
        new JoystickButton(flightstickController, Constants.Controls.macroL3).onTrue(
                new ElevatorMacroCmd(Constants.ElevatorConstants.encoderLimit, eleSub)
        );
        new JoystickButton(flightstickController, Constants.Controls.macroDown).onTrue(
                new ElevatorMacroCmd(0, eleSub)
        );
        new JoystickButton(flightstickController, Constants.Controls.elevatorUp)
                .onTrue(new ElevatorUpCmd(eleSub))
                .onFalse(new ElevatorStopCmd(eleSub));
        new JoystickButton(flightstickController, Constants.Controls.elevatorDown)
                .onTrue(new ElevatorDownCmd(eleSub))
                .onFalse(new ElevatorStopCmd(eleSub));
        new JoystickButton(flightstickController, Constants.Controls.flywheelIn)
                .onTrue(new FlywheelInCmd(chuteSub))
                .onFalse(new FlywheelStopCmd(chuteSub));
        new JoystickButton(flightstickController, Constants.Controls.flywheelOut)
                .onTrue(new FlywheelOutCmd(chuteSub))
                .onFalse(new FlywheelStopCmd(chuteSub));
        new JoystickButton(flightstickController, Constants.Controls.chuteIn).onTrue(
                new PistonExtendCmd(chuteSub, eleSub)
        );
        new JoystickButton(flightstickController, Constants.Controls.chuteOut).onTrue(
                new PistonRetractCmd(chuteSub)
        );
        new JoystickButton(flightstickController, Constants.Controls.macroShoot).onTrue(
                new ChuteMacroCmd(chuteSub, eleSub)
        );

        // A mere test
        new JoystickButton(flightstickController, 12).onTrue(
                new Command() {
                    @Override
                    public void execute() {
                        chuteSub.compressor.enableDigital();
                    }

                    @Override
                    public boolean isFinished() {
                        return true;
                    }
                }
        );
        new JoystickButton(flightstickController, 11).onTrue(
                new Command() {
                    @Override
                    public void execute() {
                        chuteSub.compressor.disable();
                        System.out.println("disabled?");
                    }

                    @Override
                    public boolean isFinished() {
                        return true;
                    }
                }
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected().andThen(new Command() {
            @Override
            public void initialize() {
                System.out.println("Hello end of auto");
            }
        });
    }
}
