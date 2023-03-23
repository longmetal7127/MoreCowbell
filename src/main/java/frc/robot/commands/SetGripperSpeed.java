package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperTrain;

public class SetGripperSpeed extends CommandBase {
    private double speed;
    private GripperTrain gripper;
    private boolean isFinished = false;
    
    public SetGripperSpeed(GripperTrain gripper, double speed) {
        addRequirements(gripper);
        this.speed = speed;
        this.gripper = gripper;
    }

    public void execute() {
        gripper.setSpeed(speed);
        isFinished = true;
    }

    public boolean isFinished() {
        return isFinished;
    }
}
