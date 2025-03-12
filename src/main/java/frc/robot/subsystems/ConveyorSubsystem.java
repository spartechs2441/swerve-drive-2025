package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
    private final SparkMax conveyor = new SparkMax(Constants.ConveyorConstants.canId, SparkLowLevel.MotorType.kBrushless);

    public ConveyorSubsystem() {
    }

    public void conveyorIn() {
        conveyor.setVoltage(Constants.ConveyorConstants.voltage);
    }

    public void conveyorOut() {
        conveyor.setVoltage(-Constants.ConveyorConstants.voltage);
    }

    public void conveyorStop() {
        conveyor.setVoltage(0);
    }
}
