package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorOutCmd extends Command {
    private final ConveyorSubsystem conveySub;
    public ConveyorOutCmd(ConveyorSubsystem conveySub) {
        this.conveySub = conveySub;
        addRequirements(conveySub);
    }

    @Override
    public void execute() {
        conveySub.conveyorOut();
    }
}
