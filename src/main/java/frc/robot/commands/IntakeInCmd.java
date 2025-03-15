package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCmd extends Command {
    private final IntakeSubsystem intakeSub;

    public IntakeInCmd(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override
    public void execute() {
        intakeSub.intakeIn();
    }
}
