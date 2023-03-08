package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Pneumatics;

public class ClawActuate extends CommandBase {
    private Pneumatics pnuematics;
    private Value direction;
    private boolean isFinished;

    public ClawActuate(Pneumatics pnuematics, Value direction) {
        this.pnuematics = pnuematics;
        this.direction = direction;

        isFinished = false;
    }

    public void execute() {
        pnuematics.set(0, direction);

        isFinished = true;
    }

    public boolean isFinished() {
        return isFinished;
    }
}
