package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class FineArmMove extends CommandBase {
    private ArmTrain arm;
    private Boolean up;
    private Boolean isFinished = false;

    public FineArmMove(ArmTrain arm, Boolean up) {
        addRequirements(arm);
        this.arm = arm;
        this.up = up;
    }

    public void execute() {
        if (this.up) {
            arm.fineMoveUp();
        } else {
            arm.fineMoveDown();
        }

        isFinished = true;
    }

    public boolean isFinished() {
        return isFinished;
    }
}
