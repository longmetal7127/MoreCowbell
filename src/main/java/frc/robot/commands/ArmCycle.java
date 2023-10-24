package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmTrain;

public class ArmCycle extends CommandBase {
    private ArmTrain arm;
    private Boolean up;
    private Boolean isFinished = false;

    private int index;
    
    public ArmCycle(ArmTrain a, Boolean ups) {
        addRequirements(a);
        this.arm = a;
        this.up = ups;

        this.index = 0;
    }

    private void incrementIndex() {
        index++;

        if (index >= Constants.ARM_HEIGHTS.length) {
          index = 0;
        }
    }

    private void decrementIndex() {
        index--;

        if (index < 0) {
            index = Constants.ARM_HEIGHTS.length - 1;
          }
    }

    public void execute() {
        if (this.up) {
            incrementIndex();
        } else {
            decrementIndex();
        }

        arm.rotate(Constants.ARM_HEIGHTS[index]);

        isFinished = true;
    }

    public boolean isFinished() {
        return this.isFinished;
    }
}
