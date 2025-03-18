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
   public static Command reefScore(String autoName, DriveSubsystem driveSub, LimelightSubsystem llSub, ElevatorSubsystem eleSub, ChuteSubsystem chuteSub) {
       return new PathPlannerAuto(autoName)
               .andThen(
                       new AutoLimelightCmd(driveSub, llSub, 1000),
                       new ElevatorMacroCmd(Constants.ElevatorConstants.encoderL3, eleSub).withDeadline(new WaitCommand(5)),
                       new FlywheelOutCmd(chuteSub).withDeadline(new WaitCommand(2))
               );

   }
}
