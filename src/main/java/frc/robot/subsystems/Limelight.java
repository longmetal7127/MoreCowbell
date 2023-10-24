// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;

public class Limelight extends SubsystemBase {
  private NetworkTable table;

  private NavX navx;
  private DriveTrain drive;

  private static MecanumDrivePoseEstimator poseEstimator;
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(1));
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1));
  
  private static MecanumDriveOdometry odometry;

  public Limelight(NavX navx, DriveTrain drive) {
    this.navx = navx;
    this.drive = drive;

    this.table = NetworkTableInstance.getDefault().getTable("limelight");

    /*poseEstimator = new MecanumDrivePoseEstimator(
        Constants.MECANUM_KINEMATICS,
        navx.getRotation2d(),
        drive.getWheelPositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs
    );*/

    odometry = new MecanumDriveOdometry(
      Constants.MECANUM_KINEMATICS,
      navx.getRotation2d(),
      drive.getWheelPositions()
    );
  }

  public Pose3d getRobotPose() {
    NetworkTableEntry posesub = null;
    if(DriverStation.getAlliance() == Alliance.Blue) {
      posesub = table.getEntry("botpose_wpiblue");
    } else {
      posesub = table.getEntry("botpose_wpired");
    }

    //NetworkTableEntry posesub = table.getEntry("botpose");

    double[] result = posesub.getDoubleArray(new double[6]);
    posesub.close();

    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);

    return new Pose3d(tran3d, r3d);
  }

  public Pose2d getRobotPose2d() {
    return getRobotPose().toPose2d();
  }

  public double getKey(String key) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).getDouble(0);
  }

  public void setPipeline(int pipelineNumber) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
  }

  @Override
  public void periodic() {
    Pose2d pose = getRobotPose2d();
    /*System.out.println("periodic - X: " + pose.getX() + " Y: " + pose.getY());
    System.out.println();*/

    if(pose.getX() != 0 && pose.getY() != 0) {
      //poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }

    //poseEstimator.updateWithTime(Timer.getFPGATimestamp(), navx.getRotation2d(), drive.getWheelPositions());
    odometry.update(navx.getRotation2d(), drive.getWheelPositions());
  }

  public Pose2d getCurrentPose() {
    //Pose2d pose = poseEstimator.getEstimatedPosition();
    Pose2d pose = odometry.getPoseMeters();

    Rotation2d rot = pose.getRotation();
    System.out.println("getCurrentPose - X: " + pose.getX() + " Y: " + pose.getY());
    System.out.println("getCurrentPose - rotation: " + rot.getDegrees());
    System.out.println("getCurrentPose - real rotation: " + navx.getRotation2d().getDegrees());
    System.out.println();
 
    return pose;
  }

  public void setPose(Pose2d pose) {
    System.out.println("setPose - X: " + pose.getX() + " Y: " + pose.getY());
    System.out.println("setPose - rotation: " + pose.getRotation().getDegrees());
    System.out.println("setPose - real rotation: " + navx.getRotation2d().getDegrees());
    System.out.println();

    //poseEstimator.resetPosition(navx.getRotation2d(), drive.getWheelPositions(), pose);
    odometry.resetPosition(navx.getRotation2d(), drive.getWheelPositions(), pose);
  }
}
