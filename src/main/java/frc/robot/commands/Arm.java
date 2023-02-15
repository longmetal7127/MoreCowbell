package frc.robot.commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Arm extends CommandBase {
    public CANSparkMax m_motor = new CANSparkMax(5, MotorType.kBrushless);
    DigitalInput toplimitSwitch = new DigitalInput(0);
    SparkMaxAbsoluteEncoder abs = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    SlewRateLimiter filter = new SlewRateLimiter(0.75);

    public Arm() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }    
    public void execute(){
        //System.out.println(abs.getPosition());
        //System.out.println(m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getPosition());
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
            m_motor.set(filter.calculate(0));
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            m_motor.set(filter.calculate(1.5));
        }

    }
}
