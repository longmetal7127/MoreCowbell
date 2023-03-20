// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class Turn extends CommandBase {
  // This constant matters!
  // Based on the momentum it could overshoot a bunch etc bla, make sure this makes a good Z turn range for the robot
  private final double kP = 0.012;

  private DriveTrain drive;
  private NavX navx;
  private double targetAngle;

  public Turn(DriveTrain drive, NavX navx, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, navx);

    this.drive = drive;
    this.navx = navx;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navx.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw, error, turnZ;

    yaw = navx.getYaw();

    error = targetAngle - yaw;
    turnZ = kP * error;

    /*System.out.println("Turn execute");
    System.out.println(yaw);
    System.out.println(turnZ);*/

    // Clamps it if too high
    if (Math.abs(turnZ) >= 0.25) {
      if (turnZ > 0) {
        turnZ = 0.25;
      } else {
        turnZ = -0.25;
      }
    }

    // Clamp if too low
    if (Math.abs(turnZ) <= 0.05) {
      if (turnZ > 0) {
        turnZ = 0.05;
      } else {
        turnZ = -0.05;
      }
    }

    //System.out.println(turnZ);

    double multiplier = (((RobotContainer.joystick.getThrottle() * -1) + 1) / 2);
    drive.drive(-RobotContainer.joystick.getY() * multiplier, RobotContainer.joystick.getX() * multiplier, turnZ * multiplier);
    //drive.drive(0, 0, turnZ);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle - navx.getYaw()) <= 0.5;
  }
}
