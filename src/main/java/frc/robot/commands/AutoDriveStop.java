package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveStop extends Command {
    private final DriveSubsystem driveSub;

    public AutoDriveStop(DriveSubsystem driveSub) {
        this.driveSub = driveSub;
        addRequirements(driveSub);
    }
    @Override
    public void initialize() {
        driveSub.formX();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
