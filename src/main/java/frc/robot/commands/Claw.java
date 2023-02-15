package frc.robot.commands;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import frc.robot.subsystems.Pnuematics;
public class Claw extends CommandBase {
    Pnuematics p;
    public Claw(Pnuematics m_pnuematics) {
        p = m_pnuematics;





    }
    public void execute() {
        p.toggle(0, kForward);

    }
}
