package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pnuematics;

public class BrakeActuate extends CommandBase {
    Pnuematics p;
    Value direction;

    public BrakeActuate(Pnuematics m_pnuematics, Value dir) {
        direction = dir;
        p = m_pnuematics;
    }

    public void execute() {
            p.toggle(2, direction);
    }
}
