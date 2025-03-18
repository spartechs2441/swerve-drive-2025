package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * Controls both the Flywheel and Conveyor to load the coral into the chute.
 * Used to lower the amount of buttons pressed when loading coral.
 */
public class CoralLoadStopCmd extends Command {
    private final ConveyorSubsystem conveySub;
    private final ChuteSubsystem chuteSub;

    public CoralLoadStopCmd(ConveyorSubsystem conveySub, ChuteSubsystem chuteSub) {
        this.conveySub = conveySub;
        this.chuteSub = chuteSub;
        addRequirements(conveySub, chuteSub);
    }

    @Override
    public void execute() {
        conveySub.conveyorStop();
        chuteSub.flywheelStop();
    }
}
