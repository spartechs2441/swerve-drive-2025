package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
    private final IntakeSubsystem intakeSub;
    private final XboxController driverController;

    public IntakeCmd(IntakeSubsystem intakeSub, XboxController driverController) {
        this.intakeSub = intakeSub;
        this.driverController = driverController;
        addRequirements(intakeSub);
    }

    @Override
    public void execute() {
        var left = driverController.getLeftTriggerAxis();
        var right = driverController.getRightTriggerAxis();
        if (left + right != 0) {
            System.out.println(left + " | " + right);
        }
       double speed = driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis();
       intakeSub.intake(speed);
    }
}
