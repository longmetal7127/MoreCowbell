// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.module.ResolutionException;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

  private DoubleArraySubscriber posesub;
  private DoubleArraySubscriber camtransub;
  private NetworkTable m_table;

  private Translation3d tran3d;
  private Rotation3d r3d;
  private Pose3d p3d;

  public Limelight() {
  }

  public Pose3d getRobotPose() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");

    posesub = m_table.getDoubleArrayTopic("botpose").subscribe(new double[] {});

    double[] result = posesub.get();
    tran3d = new Translation3d(result[0], result[1], result[2]);
    r3d = new Rotation3d(result[3], result[4], result[4]);
    p3d = new Pose3d(tran3d, r3d);
    return p3d;

  }

  @Override
  public void periodic() {

  }
}
