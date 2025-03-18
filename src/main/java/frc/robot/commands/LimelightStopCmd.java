package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightStopCmd extends Command {
    public LimelightStopCmd(LimelightSubsystem llSub) {
        addRequirements(llSub);
    }
}
