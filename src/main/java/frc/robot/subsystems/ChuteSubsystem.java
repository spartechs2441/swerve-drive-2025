package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteSubsystem extends SubsystemBase {

    private final DoubleSolenoid m_doubleSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    public ChuteSubsystem() {}

    public void extendPiston() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractPiston() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}
