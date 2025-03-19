package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightCmd extends Command {

    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem llSub;

    public AutoLimelightCmd(DriveSubsystem driveSubsystem, LimelightSubsystem llSub) {
        this.driveSubsystem = driveSubsystem;
        this.llSub = llSub;
        addRequirements(driveSubsystem, llSub);
    }

    @Override
    public void execute() {
        llSub.drive(0, driveSubsystem);
    }
}
