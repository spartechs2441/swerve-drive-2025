package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChuteSubsystem extends SubsystemBase {

    private final DoubleSolenoid doubleSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 7);
    private final SparkMax flywheel = new SparkMax(Constants.ChuteConstants.canId, SparkLowLevel.MotorType.kBrushless);
    public final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private boolean isExtended = false;

    public boolean isExtended() {
        return isExtended;
    }

    public ChuteSubsystem() {
    }

    public void extendPiston(RelativeEncoder encoder) {
        if (encoder.getPosition() >= Constants.ChuteConstants.pistonThreshold) {
            doubleSolenoid.set(DoubleSolenoid.Value.kForward);
            isExtended = true;
        }
    }

    public void retractPiston() {
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        isExtended = false;
    }

    public void flywheelIn() {
        flywheel.setVoltage(Constants.ChuteConstants.flywheelVoltage);
    }

    public void flywheelOut() {
        flywheel.setVoltage(-Constants.ChuteConstants.flywheelVoltage);
    }

    public void flywheelStop() {
        flywheel.setVoltage(0);
    }
}
