package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Pnuematics;

public class ClawActuate extends CommandBase {
    Pnuematics p;
    Value direction;

    public ClawActuate(Pnuematics m_pnuematics, Value val) {
        p = m_pnuematics;
        direction = val;
    }

    public void execute() {
        p.toggle(0, direction);
    }
}
