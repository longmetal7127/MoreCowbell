package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class ArmCycle extends CommandBase {
    private ArmTrain arm;
    private Boolean up;
    private Boolean isFinished = false;
    
    public ArmCycle(ArmTrain a, Boolean ups) {
        addRequirements(a);
        this.arm = a;
        this.up = ups;
    }

    public void execute() {
        if (this.up) {
            arm.moveUp();
        } else {
            arm.moveDown();
        }

        isFinished = true;
    }

    public boolean isFinished() {
        return isFinished;
    }
}