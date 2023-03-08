package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class ArmToPosition extends CommandBase {
    private ArmTrain arm;
    private double position;
    private Boolean isFinished = false;
    
    public ArmToPosition(ArmTrain arm, double position) {
        addRequirements(arm);
        this.arm = arm;
        this.position = position;
    }

    public void execute() {
        arm.rotate(position);
        
        isFinished = true;
    }

    public boolean isFinished() {
        return isFinished;
    }
}
