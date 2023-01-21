package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveXYZ extends CommandBase {
    private DriveTrain drive;
    double x;
    double y;
    double z;

    public DriveXYZ(DriveTrain m_Drive, double m_x, double m_y, double m_z) {
        drive = m_Drive;
        x = m_x;
        y = m_y;
        z = m_z;
    }

    public void execute() {
        drive.drive(y, x, z);
    }

}
