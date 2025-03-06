package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ChuteMacroCmd extends Command {
    private final ChuteSubsystem chuteSub;
    private final ElevatorSubsystem eleSub;
    public ChuteMacroCmd(ChuteSubsystem chuteSub, ElevatorSubsystem eleSub) {
        this.chuteSub = chuteSub;
        this.eleSub = eleSub;
        addRequirements(chuteSub);

        new PistonExtendCmd(chuteSub).andThen(
                new FlywheelOutCmd(chuteSub),
                new WaitCommand(1),
                new PistonRetractCmd(chuteSub),
                new ElevatorMacroCmd(0, eleSub)
        );
    }
}
