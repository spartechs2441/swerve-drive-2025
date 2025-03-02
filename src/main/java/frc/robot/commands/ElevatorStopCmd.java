package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStopCmd extends Command {
    private final ElevatorSubsystem eleSub;
    public ElevatorStopCmd(ElevatorSubsystem eleSub) {
        this.eleSub = eleSub;
        addRequirements(eleSub);
    }
    @Override
    public void execute() {
        eleSub.elevatorStop();
    }
}
