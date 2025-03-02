package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCmd extends Command {
    private final ElevatorSubsystem eleSub;
    public ElevatorUpCmd(ElevatorSubsystem eleSub) {
        this.eleSub = eleSub;
        addRequirements(eleSub);
    }
    @Override
    public void execute() {
        eleSub.elevatorUp();
    }
    @Override
    public void end(boolean interrupted) {
        eleSub.elevatorStop();
    }
}
