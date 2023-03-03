// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleToIntFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTrain extends SubsystemBase {
  // Define the Spark Max motor controllers for the arm train
  private int currentAdjustment = 0;
  private CANSparkMax armMotor = new CANSparkMax(Constants.armMotor, MotorType.kBrushless);
  private SparkMaxPIDController pidControl = null;
  private double heights[] = { 0, -13, -37.9, -53, -60.64, -88.0 };
  /* 
   * Ideally, heights is an array such that:
   * Index 0 is within the robot in order to hold the arm steady
   * Index 1 is the height to pick up cones and cubes; ideally this will work for both
   * Index 2 is the height to carry game pieces at; this should still be low. This is also going to work for the bottom gate drop
   * Index 3 is the middle level of the gates (should be at the height of the middle cone pole; this should work for the cubes too)
   * Index 4 is the the top level of the gates (should be at the height of the top cone pole; this should work for the cubes too)
   * Index 5 is the Player Station
   */
  private int currentHeightIndice = 0;
  // Create an encoder for the controller
  private RelativeEncoder encoder = armMotor.getEncoder();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public NetworkTable m_table;
  public DoublePublisher sub;

  public ArmTrain() {
    m_table = NetworkTableInstance.getDefault().getTable("robot");
    sub = m_table.getDoubleTopic("arm").publish();
    // Restoring Factory Defaults for motor controller
    armMotor.restoreFactoryDefaults();
    pidControl = armMotor.getPIDController();

    // PID coefficients
    kP = 0.05;
    kI = 0;
    kD = 0.05;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.6;
    kMinOutput = -0.6;

    // set PID coefficients
    pidControl.setP(kP);
    pidControl.setI(kI);
    pidControl.setD(kD);
    pidControl.setIZone(kIz);
    pidControl.setFF(kFF);
    pidControl.setOutputRange(kMinOutput, kMaxOutput);
    armMotor.setIdleMode(IdleMode.kBrake);
    pidControl.setSmartMotionMaxAccel(20,0);

  }

  public void calibrate() {

  }

  // Spark max pid controller
  // SparkMaxPIDController
  public void zero() {
    encoder.setPosition(0);
  }

  public void rotate(double angle) {
    if (angle > 90) {
      angle = 90;
    }

    double encoderValue = angle * Constants.ARM_GEAR_RATIO / 304.5;
    if (encoderValue > 0) {
      encoderValue = 0;
    }
    pidControl.setReference(encoderValue, CANSparkMax.ControlType.kPosition);
    // armMotor.set(angle);
  }

  public void moveUp() {
    currentAdjustment = 0;
    if (currentHeightIndice == heights.length - 1) {
      currentHeightIndice = 0;
    } else {
      currentHeightIndice++;
    }

    rotate(heights[currentHeightIndice]);
    System.out.println(encoder.getPosition() +"\n\n\n\n\n\n");
  }

  public void fineMoveUp() {
    currentAdjustment-= 5;
    rotate(heights[currentHeightIndice] + currentAdjustment);
    System.out.println("\n\n\n\n\n" + (heights[currentHeightIndice] + currentAdjustment) + "\n\n\n\n\n");
    System.out.println(encoder.getPosition() +"\n\n\n\n\n\n");
  }

  public void fineMoveDown() {
    currentAdjustment+= 5;
    rotate(heights[currentHeightIndice] + currentAdjustment);
    System.out.println("\n\n\n\n\n" + (heights[currentHeightIndice] + currentAdjustment) + "\n\n\n\n\n");
    System.out.println(encoder.getPosition() +"\n\n\n\n\n\n");
  }

  public void moveDown() {
    currentAdjustment = 0;
    if (currentHeightIndice == 0) {
      currentHeightIndice = heights.length - 1;
    } else {
      currentHeightIndice--;
    }
    rotate(heights[currentHeightIndice]);
    System.out.println(encoder.getPosition() +"\n\n\n\n\n\n");
  }

  public void reset() {
    rotate(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Gyro", getGyroAngle());
  }
}
