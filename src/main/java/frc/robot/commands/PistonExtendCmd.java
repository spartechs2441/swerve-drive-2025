package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;

public class PistonExtendCmd extends Command {
    ChuteSubsystem chuteSub;
    private long time;
    public PistonExtendCmd(ChuteSubsystem chuteSub) {
        this.chuteSub = chuteSub;
        this.time = System.currentTimeMillis();
;
        addRequirements(chuteSub);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - this.time) > 200;
    }

    @Override
    public void execute() {
        chuteSub.extendPiston();
    }
}
