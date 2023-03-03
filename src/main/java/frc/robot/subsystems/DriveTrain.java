// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  // Define the four Spark Max motor controllers for the drivetrain
  // There are two motor controllers for each side, one follower, and one master
  public CANSparkMax m_leftFront = new CANSparkMax(Constants.leftFront, MotorType.kBrushless);
  public CANSparkMax m_leftBack = new CANSparkMax(Constants.leftBack, MotorType.kBrushless);
  public CANSparkMax m_rightFront = new CANSparkMax(Constants.rightFront, MotorType.kBrushless);
  public CANSparkMax m_rightBack = new CANSparkMax(Constants.rightBack, MotorType.kBrushless);

  public SlewRateLimiter slewX = new SlewRateLimiter(0.75);
  public SlewRateLimiter slewY = new SlewRateLimiter(0.75);
  public SlewRateLimiter slewZ = new SlewRateLimiter(1.4);

  // Create a new DifferentialDrive object
  public MecanumDrive m_drive;

  // Create an encoder for the master controller of each side
  public RelativeEncoder m_leftEncoder = m_leftFront.getEncoder();
  public RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();

  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  double kP = 0.05;

  public DriveTrain() {
    // Restoring Factory Defaults for each motor controller
    m_leftBack.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    m_rightFront.setInverted(true);
    m_rightBack.setInverted(true);

    gyro.calibrate();

    /*
     * Since each side of the drive train has two motors geared togther,
     * we tell one of the motor controllers on each size to follow, or copy,
     * everything
     * the other on the same side motor controller does.
     */

    // Feed the DifferentialDrive the two motor controllers
    m_drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
    //m_drive.setMaxOutput(0.75);
    m_drive.setDeadband(0.05);
  }
  
  // Takes in doubles for translation and rotation(both -1 to 1)
  public void drive(double y, double x, double z) {
    // Swapping x and y because of joystick
    //m_drive.driveCartesian(y, x, z);
    m_drive.driveCartesian(slewX.calculate(y), slewY.calculate(x), slewZ.calculate(z));
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Gyro", getGyroAngle());
  }
}
