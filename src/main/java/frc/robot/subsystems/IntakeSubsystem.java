package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax hinge = new SparkMax(Constants.IntakeConstants.hingeCanId, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax intake = new SparkMax(Constants.IntakeConstants.intakeCanId, SparkLowLevel.MotorType.kBrushless);
    public IntakeSubsystem() {}

    public void hingeDown() {
        hinge.setVoltage(Constants.IntakeConstants.hingeVoltage);
    }

    public void hingeUp() {
        hinge.setVoltage(-Constants.IntakeConstants.hingeVoltage);
    }

    public void intake(double speed) {
        intake.setVoltage(Constants.IntakeConstants.intakeVoltage * speed);
    }
}
