// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTrain extends SubsystemBase {
  // Define the Spark Max motor controllers for the arm train
  private CANSparkMax armMotor = new CANSparkMax(Constants.armMotor, MotorType.kBrushless);
  private SparkMaxPIDController pidControl = null;

  // Create an encoder for the controller
  private RelativeEncoder encoder = armMotor.getEncoder();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public ArmTrain() {
    // Restoring Factory Defaults for motor controller
    armMotor.restoreFactoryDefaults();
    pidControl = armMotor.getPIDController();

    // PID coefficients
    kP = 0.05;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.4;
    kMinOutput = -0.4;

    // set PID coefficients
    pidControl.setP(kP);
    pidControl.setI(kI);
    pidControl.setD(kD);
    pidControl.setIZone(kIz);
    pidControl.setFF(kFF);
    pidControl.setOutputRange(kMinOutput, kMaxOutput);
    armMotor.setIdleMode(IdleMode.kBrake);

  }

  public void calibrate() {

  }

  // Spark max pid controller
  // SparkMaxPIDController
  public void rotate(double angle) {
    if(angle > 90) {
      angle = 90;
    }

    double encoderValue = angle * Constants.ARM_GEAR_RATIO /50;
    System.out.print("encode " + encoderValue + " actual " + encoder.getPosition());
    if(encoderValue > 0) {
      encoderValue = 0;
    }
    pidControl.setReference(encoderValue, CANSparkMax.ControlType.kPosition);
    //armMotor.set(angle);
  }

  public void reset() {
    rotate(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Gyro", getGyroAngle());
  }
}
