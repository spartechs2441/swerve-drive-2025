package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;

public class PistonExtendCmd extends Command {
    ChuteSubsystem chuteSub;
    public PistonExtendCmd(ChuteSubsystem chuteSub) {
        this.chuteSub = chuteSub;
        addRequirements(chuteSub);
    }
    @Override
    public void execute() {
        chuteSub.extendPiston();
    }
}
