//This is Limelight Code from the Old Robot
//Expect Stuff to go wrong

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCmd extends Command {
    private final XboxController joystick;
    private final LimelightSubsystem llSub;
    private final DriveSubsystem m_robotDrive;

    public LimelightCmd(LimelightSubsystem llSub, DriveSubsystem subsystem, XboxController joystick) {
        this.llSub = llSub;
        this.joystick = joystick;
        m_robotDrive = subsystem;
        addRequirements(m_robotDrive, llSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double ySpeed = -MathUtil.applyDeadband(this.joystick.getRawAxis(Constants.Controls.yMovement),
                Constants.OIConstants.kDriveDeadband);

        llSub.drive(ySpeed, m_robotDrive);
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
