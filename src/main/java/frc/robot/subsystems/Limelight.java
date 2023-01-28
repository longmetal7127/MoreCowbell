// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable m_table;
  private DoubleArraySubscriber posesub;

  public Limelight() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    posesub = m_table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
  }

  public Pose3d getRobotPose() {
    double[] result = posesub.get();

    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
    return new Pose3d(tran3d, r3d);
  }

  private LinearFilter mfX = LinearFilter.movingAverage(3);
  private LinearFilter mfY = LinearFilter.movingAverage(3);
  private LinearFilter mfZ = LinearFilter.movingAverage(3);

  public Pose3d getSmoothRobotPose() {
    double[] result = posesub.get();
    
    Translation3d tran3d = new Translation3d(
      mfX.calculate(result[0]),
      mfY.calculate(result[1]),
      mfZ.calculate(result[2])
    );
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);

    return new Pose3d(tran3d, r3d);
  }

  @Override
  public void periodic() {

  }
}