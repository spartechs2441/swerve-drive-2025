package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;

public class FlywheelStopCmd extends Command {

    private final ChuteSubsystem chuteSub;

    public FlywheelStopCmd(ChuteSubsystem chuteSub) {
        this.chuteSub = chuteSub;
        addRequirements(chuteSub);
    }

    @Override
    public void execute() {
        chuteSub.flywheelStop();
    }
}
