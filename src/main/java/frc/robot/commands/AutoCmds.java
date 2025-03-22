package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoCmds {
    public static Command reefScore(Command path, DriveSubsystem driveSub, LimelightSubsystem llSub, ElevatorSubsystem eleSub, ChuteSubsystem chuteSub) {
        // This is not supposed to be done, there are supposed to be commands in the PathPlanner auto
        // Pathplanner doesn't allow us to get the drive sub (I think) and so we need to do this terribleness
        // (because of the limelight correction that I now doubt we need)
        // It isn't *that* bad because we are really only using one type of auto: Move and score
        return path
                .andThen(
                    /*
                       new AutoLimelightCmd(driveSub, llSub).withDeadline(new WaitCommand(4)),
                       new AutoDriveStop(driveSub),
                       new PathPlannerAuto(goingLeft ? "CursedLeft" : "CursedRight"),
                       new PathPlannerAuto("CursedForward"),
                    */
                        new ElevatorMacroCmd(Constants.ElevatorConstants.encoderL3, eleSub).withDeadline(new WaitCommand(5)),
                        new FlywheelOutCmd(chuteSub).withDeadline(new WaitCommand(1)),
                        new ElevatorMacroCmd(Constants.ElevatorConstants.encoderL2, eleSub).withDeadline(new WaitCommand(5))
                );

    }
}
