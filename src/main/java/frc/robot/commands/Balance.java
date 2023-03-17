package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class Balance extends CommandBase {
    private DriveTrain drive;
    private NavX navx;

    double x;
    double y;
    Rotation3d rotation;

    public Balance(DriveTrain m_drive, NavX m_navX) {
        drive = m_drive;
        navx = m_navX;
    }

    public void execute() {
        //temp code - to be tested
        double pitch = navx.getPitch(); 
        if (pitch > 10) {
            drive.drive(0.1, 0, 0);
        }
        if (pitch < -10) {
            drive.drive(-0.1, 0, 0);
        }
    }
}
