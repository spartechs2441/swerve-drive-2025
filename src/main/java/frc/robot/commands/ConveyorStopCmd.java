package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorStopCmd extends Command {
    private final ConveyorSubsystem conveySub;

    public ConveyorStopCmd(ConveyorSubsystem conveySub) {
        this.conveySub = conveySub;
        addRequirements(conveySub);
    }

    @Override
    public void execute() {
        conveySub.conveyorStop();
    }
}
