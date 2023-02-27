package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Pneumatics;

public class ClawActuate extends CommandBase {
    Pneumatics p;
    Value direction;

    public ClawActuate(Pneumatics m_pnuematics, Value val) {
        p = m_pnuematics;
        direction = val;
    }

    public void execute() {
        p.toggle(0, direction);
    }
}
