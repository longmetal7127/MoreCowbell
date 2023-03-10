// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

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
  public MecanumDriveKinematics m_kinematics;

  // Create an encoder for the master controller of each side
  public RelativeEncoder m_leftFrontEncoder = m_leftFront.getEncoder();
  public RelativeEncoder m_rightFrontEncoder = m_rightFront.getEncoder();
  public RelativeEncoder m_leftBackEncoder = m_leftBack.getEncoder();
  public RelativeEncoder m_rightBackEncoder = m_rightBack.getEncoder();
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
    
    // Locations of the wheels relative to the robot center

    // Creating kinematics object
    
    //m_drive.setMaxOutput(0.75);
    m_drive.setDeadband(0.05);
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(m_leftFrontEncoder.getPosition(), m_rightFrontEncoder.getPosition(), m_leftBackEncoder.getPosition(), m_rightBackEncoder.getPosition());

  }

  public void resetPose(Pose2d pose) {

  }

  public void setOutputSpeeds(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheels = Constants.KINEMATICS.toWheelSpeeds(speeds);
    System.out.println(wheels.frontLeftMetersPerSecond);
    System.out.println(wheels.frontRightMetersPerSecond);
    System.out.println(wheels.rearLeftMetersPerSecond);
    System.out.println(wheels.rearRightMetersPerSecond);

    m_leftFront.set(wheels.frontLeftMetersPerSecond / 60);
    m_rightFront.set(wheels.frontRightMetersPerSecond / 60);
    m_leftBack.set(wheels.rearLeftMetersPerSecond / 60);
    m_rightBack.set(wheels.rearRightMetersPerSecond / 60);
    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
   // ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // Convert to wheel speeds
    //return speeds;
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
