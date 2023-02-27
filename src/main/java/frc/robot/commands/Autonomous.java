// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.commands.ClawActuate;
public final class Autonomous extends SequentialCommandGroup{
  /** Example static factory for an autonomous command. */
  public Autonomous(DriveTrain m_drive, Limelight m_limelight, Pneumatics m_pneumatics) {
    boolean blue = DriverStation.getAlliance() == DriverStation.Alliance.Blue; //replace with not verbose
    boolean mid = false; //side or mid starting
    if (blue && !mid/*see above*/) {
      addCommands( //replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)
        new GoTo(m_drive, m_limelight, 0, -4, 0.15, 0.05),
        new ClawActuate(m_pneumatics, Value.kForward),
        new GoTo(m_drive, m_limelight, 0, 0, 0.15, 0.05),
        new Turn(m_drive, 0.1),  
        new ClawActuate(m_pneumatics, Value.kReverse)
        /* Idea is: 
         * Move up to grid
         * drop piece
         * move back to piece on ground
         * turn
         * pick up piece
        */
      );
    } else {
      addCommands(
        
      );
    }
  }
  
  private Autonomous() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
