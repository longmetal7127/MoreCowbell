// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.commands.ClawActuate;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autonomous extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */
  enum Position {
    FAR,
    MID,
    CLOSE
  }

  public Autonomous(DriveTrain m_drive, Limelight m_limelight, Pneumatics m_pneumatics, NavX navx, ArmTrain m_arm) {
    boolean blueTeam = DriverStation.getAlliance() == DriverStation.Alliance.Blue; // replace with not verbose
    blueTeam = false; // for testing

    // NEED april tag mode to get XYZ
    Position position;
    if (m_limelight.getRobotPose().getY() < 0.27) {
      position = Position.FAR;
    } else if (m_limelight.getRobotPose().getY() < 2.05) {
      position = Position.MID;
    } else {
      position = Position.CLOSE;
    }

    System.out.println("auto blue team: " + blueTeam);
    System.out.println("auto position: " + position);
    
    if (blueTeam) {
      if (position == Position.MID) {
        addCommands(
          // replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)
          new GoTo(m_drive, m_limelight, 6.83, 1.75, 0.15, 0.05),
          new ArmToPosition(m_arm, -100),
          new ClawActuate(m_pneumatics, Value.kForward),
          new GoTo(m_drive, m_limelight, 6.83, 2.286, 0.15, 0.05),
          new GoTo(m_drive, m_limelight, 6.83, 4.265, 0.15, 0.05),
          new Balance(m_drive, navx)
          /* Idea is: 
          * Move up to grid
          * drop piece
          * move back past charging station
          * move back up
          * balance
          */
        );
      }

      if (position == Position.CLOSE) {
        addCommands(
          // replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)
          new GoTo(m_drive, m_limelight, 6.83, 3.36, 0.15, 0.05),
          new ArmToPosition(m_arm, -100),
          new ClawActuate(m_pneumatics, Value.kForward),
          new GoTo(m_drive, m_limelight, 1.12, 2.87, 0.15, 0.05),
          new Turn(m_drive, navx, 180),  
          new ClawActuate(m_pneumatics, Value.kReverse),
          new Turn(m_drive, navx, 180)
          /* Idea is: 
          * Move up to grid
          * drop piece
          * move back to piece on ground
          * turn
          * pick up piece
          */
        );
      }

      if (position == Position.FAR) {
        addCommands(
          // replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)
          new GoTo(m_drive, m_limelight, 6.83, 0.023, 0.15, 0.05),
          new ArmToPosition(m_arm, -100),
          new ClawActuate(m_pneumatics, Value.kForward),
          new GoTo(m_drive, m_limelight, 1.12, 0.2499, 0.15, 0.05),
          new Turn(m_drive, navx, 180),  
          new ClawActuate(m_pneumatics, Value.kReverse),
          new Turn(m_drive, navx, 180)
          /* Idea is: 
          * Move up to grid
          * drop piece
          * move back to piece on ground
          * turn
          * pick up piece
          */
        );
      }
    } else {
      if (position == Position.MID) {
        addCommands(
          // replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)
          new GoTo(m_drive, m_limelight, 6.83, 1.75, 0.15, 0.05),
          new ArmToPosition(m_arm, -100),
          new ClawActuate(m_pneumatics, Value.kForward),
          new GoTo(m_drive, m_limelight, 6.83, 2.286, 0.15, 0.05),
          new GoTo(m_drive, m_limelight, 6.83, 4.265, 0.15, 0.05),
          new Balance(m_drive, navx)
          /* Idea is: 
          * Move up to grid
          * drop piece
          * move back past charging station
          * move back up
          * balance
          */
        );
      }

      if (position == Position.CLOSE) {
        addCommands(
          // replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)
          new GoTo(m_drive, m_limelight, -6.83, 3.36, 0.15, 0.05),
          new ArmToPosition(m_arm, -100),
          new ClawActuate(m_pneumatics, Value.kForward),
          new GoTo(m_drive, m_limelight, -1.12, 2.87, 0.15, 0.05),
          new Turn(m_drive, navx, 180),  
          new ClawActuate(m_pneumatics, Value.kReverse),
          new Turn(m_drive, navx, 180)
          /* Idea is: 
          * Move up to grid
          * drop piece
          * move back to piece on ground
          * turn
          * pick up piece
          */
        );
      }

      if (position == Position.FAR) {
        addCommands(
          // replace target coords with real coords. probably need to change some ordering (parallel turning and driving?)

          new ArmToPosition(m_arm, -100),
          new WaitCommand(5),
          new ClawActuate(m_pneumatics, Value.kReverse)
          //new GoTo(m_drive, m_limelight, 6.83, 0.023, 0.15, 0.05)
          // new ClawActuate(m_pneumatics, Value.kForward),
          // new GoTo(m_drive, m_limelight, -1.12, 0.2499, 0.15, 0.05),
          // new Turn(m_drive, navx, 180),  
          // new ClawActuate(m_pneumatics, Value.kReverse),
          // new Turn(m_drive, navx, 180)
          /* Idea is: 
          * Move up to grid
          * drop piece
          * move back to piece on ground
          * turn
          * pick up piece
          */
        );
      }
    }
  }
  
  private Autonomous() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
