package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PistonExtendCmd extends Command {
    ChuteSubsystem chuteSub;
    ElevatorSubsystem eleSub;
    private long time;
    public PistonExtendCmd(ChuteSubsystem chuteSub, ElevatorSubsystem eleSub) {
        this.chuteSub = chuteSub;
        this.eleSub = eleSub;
        this.time = System.currentTimeMillis();
;
        addRequirements(chuteSub, eleSub);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - this.time) > 200;
    }

    @Override
    public void execute() {
        chuteSub.extendPiston(eleSub.getElevatorEncoder());
    }
}
