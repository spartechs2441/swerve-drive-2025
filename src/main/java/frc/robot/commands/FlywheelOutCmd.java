package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;

public class FlywheelOutCmd extends Command {
    private final ChuteSubsystem chuteSub;
    public FlywheelOutCmd(ChuteSubsystem chuteSub) {
        this.chuteSub = chuteSub;
        addRequirements(chuteSub);
    }

    @Override
    public void execute() {
        chuteSub.flywheelOut();
    }
}
