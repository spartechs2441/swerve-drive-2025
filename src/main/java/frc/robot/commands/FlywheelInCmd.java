package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;

public class FlywheelInCmd extends Command {
    private final ChuteSubsystem chuteSub;
    public FlywheelInCmd(ChuteSubsystem chuteSub) {
        this.chuteSub = chuteSub;
        addRequirements(chuteSub);
    }

    @Override
    public void execute() {
        chuteSub.flywheelIn();
    }
}
