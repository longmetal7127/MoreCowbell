package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperTrain extends SubsystemBase {
    private CANSparkMax motorLeft = new CANSparkMax(Constants.gripperLeftId, MotorType.kBrushless);
    private CANSparkMax motorRight = new CANSparkMax(Constants.gripperRightId, MotorType.kBrushless);

    public GripperTrain() {
        motorRight.setInverted(true);
    }

    public void setSpeed(double speed) {
        motorLeft.set(speed);
        motorRight.set(speed);
    }
}
