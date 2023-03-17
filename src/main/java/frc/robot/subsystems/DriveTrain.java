// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  // Define the four Spark Max motor controllers for the drivetrain
  // There are two motor controllers for each side, one follower, and one master
  private CANSparkMax m_leftFront = new CANSparkMax(Constants.leftFront, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(Constants.leftBack, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(Constants.rightFront, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(Constants.rightBack, MotorType.kBrushless);

  private SlewRateLimiter slewX = new SlewRateLimiter(0.75);
  private SlewRateLimiter slewY = new SlewRateLimiter(0.75);
  private SlewRateLimiter slewZ = new SlewRateLimiter(1.4);

  // Create a new DifferentialDrive object
  private MecanumDrive m_drive;

  // Create an encoder for the master controller of each side
  private RelativeEncoder m_leftFrontEncoder = m_leftFront.getEncoder();
  private RelativeEncoder m_rightFrontEncoder = m_rightFront.getEncoder();
  private RelativeEncoder m_leftBackEncoder = m_leftBack.getEncoder();
  private RelativeEncoder m_rightBackEncoder = m_rightBack.getEncoder();

  private SparkMaxPIDController m_leftFrontPID;
  private SparkMaxPIDController m_leftBackPID;
  private SparkMaxPIDController m_rightFrontPID;
  private SparkMaxPIDController m_rightBackPID;

  private SimpleMotorFeedforward mFF = new SimpleMotorFeedforward(0.1, 2.8, 1.5);

  public DriveTrain() {
    // Restoring Factory Defaults for each motor controller
    m_leftBack.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    
    // Invert right side
    m_rightFront.setInverted(true);
    m_rightBack.setInverted(true);
    
    // Set zero position
    resetEncoders();

    // Set position conversion factors
    m_leftFrontEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);
    m_rightFrontEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);
    m_leftBackEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);
    m_rightBackEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);

    // Set velocity conversion factors
    m_leftFrontEncoder.setVelocityConversionFactor(Constants.kEncoderVelocityFactor);
    m_rightFrontEncoder.setVelocityConversionFactor(Constants.kEncoderVelocityFactor);
    m_leftBackEncoder.setVelocityConversionFactor(Constants.kEncoderVelocityFactor);
    m_rightBackEncoder.setVelocityConversionFactor(Constants.kEncoderVelocityFactor);

    // Grab PID refernces and set them up
    m_leftFrontPID = m_leftFront.getPIDController();
    m_leftBackPID = m_leftBack.getPIDController();
    m_rightFrontPID = m_rightFront.getPIDController();
    m_rightBackPID = m_rightBack.getPIDController();

    m_leftFrontPID.setP(0);
    m_leftFrontPID.setI(0);
    m_leftFrontPID.setD(0);
    m_leftFrontPID.setOutputRange(-1, 1);

    m_leftBackPID.setP(0);
    m_leftBackPID.setI(0);
    m_leftBackPID.setD(0);
    m_leftBackPID.setOutputRange(-1, 1);

    m_rightFrontPID.setP(0);
    m_rightFrontPID.setI(0);
    m_rightFrontPID.setD(0);
    m_rightFrontPID.setOutputRange(-1, 1);

    m_rightBackPID.setP(0);
    m_rightBackPID.setI(0);
    m_rightBackPID.setD(0);
    m_rightBackPID.setOutputRange(-1, 1);

    // Feed the DifferentialDrive the two motor controllers
    m_drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
    m_drive.setSafetyEnabled(false);
    //m_drive.setMaxOutput(0.75);
    m_drive.setDeadband(0.05);
  }

  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    m_leftBackEncoder.setPosition(0);
    m_rightBackEncoder.setPosition(0);
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      m_leftFrontEncoder.getPosition(),
      m_rightFrontEncoder.getPosition(),
      m_leftBackEncoder.getPosition(),
      m_rightBackEncoder.getPosition()
    );
  }

  public void setOutputSpeeds(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheels = Constants.MECANUM_KINEMATICS.toWheelSpeeds(speeds);
    wheels.desaturate(Constants.PATH_MAX_VELOCITY);

    m_leftFrontPID.setReference(
      wheels.frontLeftMetersPerSecond,
      ControlType.kVelocity,
      0,
      mFF.calculate(wheels.frontLeftMetersPerSecond),
      ArbFFUnits.kVoltage
    );

    m_leftBackPID.setReference(
      wheels.rearLeftMetersPerSecond,
      ControlType.kVelocity,
      0,
      mFF.calculate(wheels.rearLeftMetersPerSecond),
      ArbFFUnits.kVoltage
    );

    m_rightFrontPID.setReference(
      wheels.frontRightMetersPerSecond,
      ControlType.kVelocity,
      0,
      mFF.calculate(wheels.frontRightMetersPerSecond),
      ArbFFUnits.kVoltage
    );

    m_rightBackPID.setReference(
      wheels.rearRightMetersPerSecond,
      ControlType.kVelocity,
      0,
      mFF.calculate(wheels.rearRightMetersPerSecond),
      ArbFFUnits.kVoltage
    );
  }

  // Takes in doubles for translation and rotation(both -1 to 1)
  public void drive(double y, double x, double z) {
    // Swapping x and y because of joystick
    //m_drive.driveCartesian(y, x, z);
    m_drive.driveCartesian(slewX.calculate(y), slewY.calculate(x), slewZ.calculate(z));
  }

  @Override
  public void periodic() {
  }
}
