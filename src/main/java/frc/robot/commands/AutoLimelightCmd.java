package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightCmd extends Command {

    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem llSub;
    private final long time;
    private final long start;

    public AutoLimelightCmd(DriveSubsystem driveSubsystem, LimelightSubsystem llSub, long time) {
        this.driveSubsystem = driveSubsystem;
        this.llSub = llSub;
        this.time = time;
        this.start = System.currentTimeMillis();
        addRequirements(driveSubsystem, llSub);
    }

    @Override
    public void execute() {
        llSub.drive(0, driveSubsystem);
    }

    @Override
    public boolean isFinished() {
        return start + time < System.currentTimeMillis();
    }
}
