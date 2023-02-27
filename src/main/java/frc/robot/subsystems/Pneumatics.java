package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics extends SubsystemBase {
    Compressor comp = new Compressor(6, PneumaticsModuleType.REVPH);

    DoubleSolenoid claw = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, 3, 2);
    DoubleSolenoid bfm = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, 1, 0);
    DoubleSolenoid brake = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, 4, 5);
    DoubleSolenoid items[] = { claw, bfm, brake };

    public Pneumatics() {
        comp.enableDigital();

    }

    public void toggle(int id, Value value) {
        items[id].set(value);
    }

}
