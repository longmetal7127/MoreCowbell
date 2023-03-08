package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class BrakeActuate extends CommandBase {
    Pneumatics p;
    Value direction;

    public BrakeActuate(Pneumatics m_pnuematics, Value dir) {
        direction = dir;
        p = m_pnuematics;
    }

    public void execute() {
        p.set(2, direction);
    }
}
