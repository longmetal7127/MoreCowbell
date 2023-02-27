package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Turn extends CommandBase {
    private DriveTrain drive;
    private double velocity;

    public Turn(DriveTrain m_drive, double velocity) {
        this.drive = m_drive;
        this.velocity = velocity;
    }

    public void execute() {
        drive.drive(0, 0, velocity);
    }
}
