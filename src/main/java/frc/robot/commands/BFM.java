package frc.robot.commands;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import frc.robot.subsystems.Pnuematics;

public class BFM extends CommandBase {
    Pnuematics p;
    Value direction;

    public BFM(Pnuematics m_pnuematics, Value val) {
        p = m_pnuematics;
        direction = val;

    }

    public void execute() {
        p.toggle(1, direction);
    }
}
