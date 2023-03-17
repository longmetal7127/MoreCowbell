// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Drive train
  public static int leftFront = 3; 
  public static int leftBack = 4;
  public static int rightFront = 1;
  public static int rightBack = 2;

  // Arm motor controller
  public static int armMotor = 5;

  // Driverstation ports 
  public static int joystickPort = 0; // USB port on driver station computer
  public static int xboxPort = 1;

  // Arm gear ratio
  public static int ARM_BIG_SPROCKET = 60;
  public static int ARM_SMALL_SPROCKET = 16;
  public static int ARM_GEARBOX_RATIO = 100;
  public static int ARM_GEAR_RATIO = 100 * (ARM_BIG_SPROCKET / ARM_SMALL_SPROCKET);

  // Conversion Factors
  public static final double kWheelDiameterMeters = 0.1524; // 6 inch
  public static final double kEncoderGearRatio = 12.75; // gear ratio
  public static final double kEncoderDistancePerPulseMeters = (kWheelDiameterMeters * Math.PI) / kEncoderGearRatio;
  public static final double kEncoderVelocityFactor = kEncoderDistancePerPulseMeters / 60.0;

  // Kinematics
  // Make sure this is centric to the drive train, not the robot
  public static MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(
    new Translation2d(0.269, 0.323),
    new Translation2d(0.269, -0.323),
    new Translation2d(-0.269, 0.323),
    new Translation2d(-0.269, -0.323)
  );

  // Path
  public static double PATH_MAX_VELOCITY = 1.5;
  public static double PATH_MAX_ACCELERATION = 0.4;

  // Arm heights for cycling and auto mode
  public static final double ARM_HEIGHTS[] = { 0, -0.2, -20.2, -85.2, -130 };
}
