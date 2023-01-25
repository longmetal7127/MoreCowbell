// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
public final class Autonomous extends SequentialCommandGroup{
  /** Example static factory for an autonomous command. */
  public Autonomous(DriveTrain m_drive, Limelight m_limelight) {

    double x = 0.5; //drives forward
    double y = 0;
    double z = 0;
    addCommands(
      new BotPoseOrient(m_drive, m_limelight )
    );
  }
  
  private Autonomous() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
