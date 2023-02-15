package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Pnuematics extends SubsystemBase {
    DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
    DoubleSolenoid bfm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    DoubleSolenoid items[] = {claw, bfm};
    public Pnuematics() {

    }
    
    public void toggle(int id, Value value) {
        items[id].set(value);
    }
    
}
