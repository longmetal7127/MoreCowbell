package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class Balance extends CommandBase {
    private DriveTrain drive;
    double x;
    double y;
    Rotation3d rotation;
    private NavX navx;

    public Balance(DriveTrain m_Drive, NavX m_NavX) {
        drive = m_Drive;
        navx = m_NavX;
    }

    public void execute() {
        //temp code - to be tested
        double pitch = NavX.getPitch(); 
        if (pitch > 10) {
            drive.drive(0.1, 0, 0);
        }
        if (pitch < -10) {
            drive.drive(-0.1, 0, 0);
        }
    }
}
