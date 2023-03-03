package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class ZeroArm extends CommandBase {
    private ArmTrain arm;

    public ZeroArm(ArmTrain a) {
        addRequirements(a);
        this.arm = a;
    }

    public void execute() {
        arm.zero(); 
    }
}
