package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Pneumatics;

public class ClawActuate extends CommandBase {
    private Pneumatics pnuematics;
    private Value direction;
    private boolean shouldFinish;
    private boolean isFinished;

    public ClawActuate(Pneumatics pnuematics, Value direction, boolean shouldFinish) {
        this.pnuematics = pnuematics;
        this.direction = direction;
        this.shouldFinish = shouldFinish;

        isFinished = false;
    }

    public void execute() {
        pnuematics.set(0, direction);
        if(shouldFinish) {
            isFinished = true;
        }
    }

    public boolean isFinished() {
        return isFinished;
    }
}
