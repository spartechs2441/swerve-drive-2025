package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMacroCmd extends Command {
    private final double target;
    private final ElevatorSubsystem eleSub;

    public ElevatorMacroCmd(double target, ElevatorSubsystem eleSub) {
        this.target = target;
        this.eleSub = eleSub;
        addRequirements(eleSub);
    }
    @Override
    public void execute() {
        eleSub.macro(target);
    }
}
