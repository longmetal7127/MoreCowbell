// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Currency;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTrain extends SubsystemBase {
  // Define the Spark Max motor controllers for the arm train
  private CANSparkMax armMotor = new CANSparkMax(Constants.armMotor, MotorType.kBrushless);
  private SparkMaxPIDController pidControl;

  // Create an encoder for the controller
  private RelativeEncoder encoder = armMotor.getEncoder();

  // Arm PID
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public ArmTrain() {
    // Restoring Factory Defaults for motor controller
    armMotor.restoreFactoryDefaults();
    pidControl = armMotor.getPIDController();

    // PID coefficients
    // If you touch these values, be VERY CAREFUL. We tried making kI 0.005 and it
    // almost broke the arm.
    kP = 0.05;
    kI = 0;
    kD = 0.05;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    pidControl.setP(kP);
    pidControl.setI(kI);
    pidControl.setD(kD);
    pidControl.setIZone(kIz);
    pidControl.setFF(kFF);
    pidControl.setOutputRange(kMinOutput, kMaxOutput);
    pidControl.setSmartMotionMaxAccel(5, 0);
  }

  public double convertAngleToEncoderValue(double angle) {
    return angle;
  }

  public void rotate(double angle) {
    if (angle < Constants.MAX_ARM_HEIGHT) {
      angle = Constants.MAX_ARM_HEIGHT;
    }

    double encoderValue = convertAngleToEncoderValue(angle);

    pidControl.setReference(encoderValue, ControlType.kPosition);
  }

  public boolean checkAngle(double angle) {
    if (angle > 90) {
      angle = 90;
    }

    double encoderValue = convertAngleToEncoderValue(angle);
    double encoderPosition = encoder.getPosition();

    double tolerance = 3;

    return encoderPosition > (encoderValue - tolerance) && encoderPosition < (encoderValue + tolerance);
  }

  public void fineMoveUp() {
    rotate(encoder.getPosition() - Constants.ARM_FINE_ADJUST);
  }

  public void fineMoveDown() {
    rotate(encoder.getPosition() + Constants.ARM_FINE_ADJUST);
  }

  @Override
  public void periodic() {
    if (armMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
      encoder.setPosition(0);
    }
  }
}
