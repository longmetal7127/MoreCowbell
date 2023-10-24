package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperTrain extends SubsystemBase {
    private CANSparkMax gripperMotors = new CANSparkMax(Constants.gripperMotorsId, MotorType.kBrushed);

    public GripperTrain() {
        gripperMotors.set(Constants.GRIPPER_FORWARD_SPEED);
    }

    public void setSpeed(double speed) {
        gripperMotors.set(speed);
    }
}
