package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCmd extends Command {
    private final ElevatorSubsystem eleSub;

    public ElevatorDownCmd(ElevatorSubsystem eleSub) {
        this.eleSub = eleSub;
        addRequirements(eleSub);
    }

    @Override
    public void execute() {
        eleSub.elevatorDown();
    }

    @Override
    public void end(boolean interrupted) {
        eleSub.elevatorStop();
    }
}
