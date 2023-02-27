package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class BFMActuate extends CommandBase {
    Pneumatics p;
    Value direction;

    public BFMActuate(Pneumatics m_pnuematics, Value val) {
        p = m_pnuematics;
        direction = val;
    }

    public void execute() {
        p.toggle(1, direction);
    }
}
