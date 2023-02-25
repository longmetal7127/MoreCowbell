package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class ArmMove extends CommandBase {
    private ArmTrain arm;
    private Boolean up;
private Boolean isFinished = false;
    public ArmMove(ArmTrain a, Boolean ups) {
        addRequirements(a);
        this.arm = a;
        this.up = ups;
        System.out.println("this should run once");

    }
   // @Override
    public void execute() {
        System.out.println("ArmMove.java");
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
