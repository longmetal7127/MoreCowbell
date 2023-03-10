// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;

public class Limelight extends SubsystemBase {
  private NetworkTable m_table;
  // private DoubleArraySubscriber posesub;
  private static MecanumDrivePoseEstimator poseEstimator;
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5));
  private NavX navX;
  private DriveTrain drive;

  public Limelight(NavX navx, DriveTrain driveTrain) {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
    this.navX = navx;
    this.drive = driveTrain;
    /*
     * MecanumDriveKinematics kinematics,
     * Rotation2d gyroAngle,
     * MecanumDriveWheelPositions wheelPositions,
     * Pose2d initialPoseMeters,
     * Matrix<N3, N1> stateStdDevs,
     * Matrix<N3, N1> visionMeasurementStdDevs) {
     */
    poseEstimator = new MecanumDrivePoseEstimator(
        Constants.KINEMATICS,
        navx.getRotation2d(),
        driveTrain.getWheelPositions(),
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)),
        stateStdDevs,
        visionMeasurementStdDevs);

  }

  public Pose3d getRobotPose() {
    NetworkTableEntry posesub = m_table.getEntry("botpose");

    double[] result = posesub.getDoubleArray(new double[0]);

    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    posesub.close();
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
    return new Pose3d(tran3d, r3d);
  }

  public Pose2d getRobotPose2d() {
    NetworkTableEntry posesub = m_table.getEntry("botpose_wpired");
    double[] result = posesub.getDoubleArray(new double[6]);
    posesub.close();

    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);

    return new Pose3d(tran3d, r3d).toPose2d();
  }

  public double getKey(String key) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).getDouble(0);
  }

  public void setPipeline(int pipelineNumber) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
  }

  private LinearFilter mfX = LinearFilter.movingAverage(3);
  private LinearFilter mfY = LinearFilter.movingAverage(3);
  private LinearFilter mfZ = LinearFilter.movingAverage(3);

  public Pose3d getSmoothRobotPose() {
    NetworkTableEntry posesub = m_table.getEntry("botpose_wpired");

    double[] result = posesub.getDoubleArray(new double[0]);

    Translation3d tran3d = new Translation3d(
        mfX.calculate(result[0]),
        mfY.calculate(result[1]),
        mfZ.calculate(result[2]));
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);

    return new Pose3d(tran3d, r3d);
  }

  @Override
  public void periodic() {
    poseEstimator.addVisionMeasurement(getRobotPose2d(), Timer.getFPGATimestamp());
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), navX.getRotation2d(), drive.getWheelPositions());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(navX.getRotation2d(), drive.getWheelPositions(), pose);
  }
}
