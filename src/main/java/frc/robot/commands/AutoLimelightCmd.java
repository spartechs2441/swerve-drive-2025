package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightCmd extends Command {

    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem llSub;
    private final double y;

    public AutoLimelightCmd(DriveSubsystem driveSubsystem, LimelightSubsystem llSub, double y) {
        this.driveSubsystem = driveSubsystem;
        this.llSub = llSub;
        this.y = y;
        addRequirements(driveSubsystem, llSub);
    }

    @Override
    public void execute() {
        llSub.drive(y, driveSubsystem);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
