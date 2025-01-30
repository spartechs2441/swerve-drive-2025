package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DriveSubsystem

class TareCmd(private val driveSub: DriveSubsystem) : Command() {
    init {
        addRequirements(driveSub)
    }

    override fun execute() {
        driveSub.tare()
    }

    override fun isFinished(): Boolean {
        return true
    }
}
