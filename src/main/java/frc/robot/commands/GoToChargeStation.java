package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class GoToChargeStation extends CommandBase {
    private DriveTrain drive;
    double x;
    double y;
    Rotation3d rotation;
    private Limelight lime;
    private Pose3d botpose;

    // Charging station bounds
    double STATION_RIGHT_Y = -2.45;
    double STATION_LEFT_Y = 0.05;
    double STATION_FRONT_X = 5.3;
    double STATION_BACK_X = 3.35;

    public GoToChargeStation(DriveTrain m_Drive, Limelight m_l) {
        drive = m_Drive;
        // makes x and y the botpose
        lime = m_l;

    }

    public void execute() {
        botpose = lime.getRobotPose();
        x = botpose.getX();
        y = botpose.getY();
        rotation = botpose.getRotation();
        /*
        double targetAngle = 0;
        System.out.println(rotation);
        System.out.println(rotation.getAngle());
        
        if (Math.abs(targetAngle - angle) >= 0.5) {
            System.out.println(-(targetAngle - angle) / 15);
            drive.drive(0, 0, -(targetAngle - angle) / 15);
        }
        */
        if (Math.abs((STATION_FRONT_X + STATION_BACK_X / 2) - x) >= 0.5 || Math.abs((STATION_LEFT_Y + STATION_RIGHT_Y / 2) - y) >= 0.5) {
            drive.drive(((STATION_FRONT_X + STATION_BACK_X / 2) - x) / 10,
                    ((STATION_LEFT_Y + STATION_RIGHT_Y / 2) - y) / 10, 0);
        }
    }
}
