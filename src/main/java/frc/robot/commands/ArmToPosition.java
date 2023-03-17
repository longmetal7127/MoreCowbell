package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class ArmToPosition extends CommandBase {
    private ArmTrain arm;
    private double position;
    
    public ArmToPosition(ArmTrain arm, double position) {
        addRequirements(arm);
        this.arm = arm;
        this.position = position;
    }

    public void execute() {
        arm.rotate(position);
    }

    public boolean isFinished() {
        return arm.checkAngle(position);
    }
}
