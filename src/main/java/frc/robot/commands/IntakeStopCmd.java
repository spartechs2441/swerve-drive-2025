package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCmd extends Command {
    private final IntakeSubsystem intakeSub;

    public IntakeStopCmd(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override
    public void execute() {
        intakeSub.intakeStop();
    }
}
