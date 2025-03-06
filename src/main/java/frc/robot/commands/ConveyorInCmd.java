package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorInCmd extends Command {
    private final ConveyorSubsystem conveySub;
    public ConveyorInCmd(ConveyorSubsystem conveySub) {
        this.conveySub = conveySub;
        addRequirements(conveySub);
    }

    @Override
    public void execute() {
        conveySub.conveyorIn();
    }
}
