// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class Turn extends CommandBase {
  /** Creates a new GyroTurn. */
  private DriveTrain drive;
private NavX nav;
  double kP = 0.1;
  double targetAngle;

  boolean finished = false;

  public Turn(DriveTrain m_drive, NavX navx, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = m_drive;
    nav = navx;
    targetAngle = a;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nav.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error, turnPower;

    while (Math.abs(targetAngle - nav.getYaw()) > 2) {
      error = targetAngle - nav.getYaw();
      turnPower = kP * error;

      // Clamps it if too high
      if (Math.abs(turnPower) >= 0.25) {
        if (turnPower > 0) {
          turnPower = 0.25;
        } else {
          turnPower = -0.25;
        }
      }

      System.out.println(turnPower);

      drive.drive(0, 0, turnPower);
    }

    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    nav.resetYaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
