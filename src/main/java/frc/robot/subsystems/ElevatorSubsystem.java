package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevator = new SparkMax(Constants.ElevatorConstants.canId, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder;
    private ChuteSubsystem chuteSub;

    public ElevatorSubsystem(ChuteSubsystem chuteSub) {
        this.chuteSub = chuteSub;
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
        double position = elevatorEncoder.getPosition();
        if (Math.abs(position - desired) < Constants.ElevatorConstants.gracePeriod) {
            elevatorStop();
            return;
        }

        // FIXME: RE-ENABLE WHEN PISTON IS BACK
//        if (chuteSub.isExtended()) return;
        if (position <= desired) {
            elevatorUp();
        } else if (position > desired) {
            elevatorDown();
        }
    }

    private void safeUp() {
        boolean isSafe = elevatorEncoder.getPosition() < Constants.ElevatorConstants.encoderLimit;
        if (isSafe) {
            elevator.setVoltage(Constants.ElevatorConstants.voltage);
        } else {
            elevatorStop();
        }
        SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getPosition());
    }

    private void safeDown() {
        boolean isSafe = elevatorEncoder.getPosition() > -10;
        if (isSafe) {
            elevator.setVoltage(-Constants.ElevatorConstants.voltage);
        } else {
            elevatorStop();
        }
        SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getPosition());
    }

    public RelativeEncoder getElevatorEncoder() {
        return elevatorEncoder;
    }
}
