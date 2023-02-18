package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;

public class RotateArm extends CommandBase {
    private ArmTrain arm;
    private double angle;
    private XboxController xbox;

    public RotateArm(ArmTrain a, double theta, XboxController x) {
        addRequirements(a);
        this.arm = a;
        this.angle = theta;
        this.xbox = x;
    }

    public void execute() {
        arm.rotate(angle);
        
    }
}
