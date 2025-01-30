package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.subsystems.DriveSubsystem
import kotlin.math.max
import kotlin.math.min

class LockRotation(val m_robotDrive: DriveSubsystem, private val targetRot: Rotation2d) : Command() {
    init {
        addRequirements(m_robotDrive)
    }

    /**
     * TODO: Make this a log curve to get rid of overturning
     */
    override fun execute() {
        var difference = (targetRot.degrees % 360
                - m_robotDrive.gyroRotation().degrees % 360)
        if (difference > 180) {
            difference -= 360.0
        } else if (difference < -180) {
            difference += 360.0
        }
        difference = (difference / 180)
        val MIN_SPEED = 0.02
        if (difference > 0) {
            difference = max(difference, MIN_SPEED)
        } else if (difference < 0) {
            difference = min(difference, -MIN_SPEED)
        }

        if (difference > 1 || difference < -1) {
            Robot.Companion.errorAssert(
                ("""Calculated rotation cannot be > 1, < -1
Calculated value: $difference target: ${targetRot.degrees % 360} actual: ${m_robotDrive.gyroRotation().degrees % 360}""")
            )
        }
        m_robotDrive.drive(0.0, 0.0, difference, true)
    }

    override fun isFinished(): Boolean {
        return false
    }
}
