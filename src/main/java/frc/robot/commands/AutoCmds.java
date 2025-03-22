package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoCmds {
   public static Command reefScore(Command path, boolean goingLeft, DriveSubsystem driveSub, LimelightSubsystem llSub, ElevatorSubsystem eleSub, ChuteSubsystem chuteSub) {
       return path
               .andThen(
//                       new AutoLimelightCmd(driveSub, llSub).withDeadline(new WaitCommand(4)),
                       new AutoDriveStop(driveSub),
//                       new PathPlannerAuto(goingLeft ? "CursedLeft" : "CursedRight"),
//                       new PathPlannerAuto("CursedForward"),
                       new ElevatorMacroCmd(Constants.ElevatorConstants.encoderL3, eleSub).withDeadline(new WaitCommand(5)),
                       new FlywheelOutCmd(chuteSub).withDeadline(new WaitCommand(1)),
                       new ElevatorMacroCmd(Constants.ElevatorConstants.encoderL2, eleSub).withDeadline(new WaitCommand(5))
               );

   }
}
