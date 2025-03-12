package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class HingeUpCmd extends Command {
    private final IntakeSubsystem intakeSub;

    public HingeUpCmd(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

//    @Override
//    public void execute() {
//        intakeSub.hingeUp();
//    }
}
