package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevator = new SparkMax(Constants.ElevatorConstants.canId, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder;
    public DigitalInput limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchDIo);


    public ElevatorSubsystem() {
        this.elevatorEncoder = elevator.getEncoder();
    }

    public void elevatorUp() {
        safeUp();
    }

    public void elevatorDown() {
        safeDown();
    }

    public void elevatorStop() {
        elevator.setVoltage(0);
    }

    public void macro(double desired) {
        final double tolerance = 10;
        double position = elevatorEncoder.getPosition();
        if (Math.abs(position - desired) < tolerance) {
            elevatorStop();
            return;
        }

        if (position <= desired) {
            elevatorUp();
        } else if (position > desired) {
            elevatorDown();
        }
    }

    private void safeUp() {
        boolean isSafe = elevatorEncoder.getPosition() < Constants.ElevatorConstants.encoderLimit && !limitSwitch.get();
        if (isSafe) {
            elevator.setVoltage(Constants.ElevatorConstants.voltage);
        } else {
            elevatorStop();
        }
        System.out.println(elevatorEncoder.getPosition());
    }

    private void safeDown() {
        boolean isSafe = elevatorEncoder.getPosition() > -10;
        if (isSafe) {
            elevator.setVoltage(-Constants.ElevatorConstants.voltage);
        } else {
            elevatorStop();
        }
        System.out.println(elevatorEncoder.getPosition());
    }

    public RelativeEncoder getElevatorEncoder() {
        return elevatorEncoder;
    }
}
