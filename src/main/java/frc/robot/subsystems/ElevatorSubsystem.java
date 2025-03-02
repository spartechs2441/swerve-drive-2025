package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    // TODO: Add these in constants
    private final SparkMax elevator = new SparkMax(Constants.ElevatorConstants.canId, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder;
    public DigitalInput limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchDIo);

    public ElevatorSubsystem() {
        this.elevatorEncoder = elevator.getEncoder();
    }

    public void elevatorUp() {
        if (elevatorEncoder.getPosition() < Constants.ElevatorConstants.encoderLimit
                && !limitSwitch.get()) {
            elevator.setVoltage(Constants.ElevatorConstants.voltage);
        } else {
            elevator.setVoltage(0);
        }
    }
    public void elevatorDown() {
        if (elevatorEncoder.getPosition() > -Constants.ElevatorConstants.encoderLimit
                && !limitSwitch.get()) {
            elevator.setVoltage(-Constants.ElevatorConstants.voltage);
        } else {
            elevator.setVoltage(0);
        }
    }
    public void elevatorStop() {
        elevator.setVoltage(0);
    }
}
