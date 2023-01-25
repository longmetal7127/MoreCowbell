package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class BotPoseOrient extends CommandBase {
    private DriveTrain drive;
    double x;
    double y;
    private Limelight lime;
    private Pose3d botpose;
    double STATION_RIGHT_Y = -2.45;
    double STATION_LEFT_Y = 0.05;
    double STATION_FRONT_X = 5.3;
    double STATION_BACK_X = 3.35;

    public BotPoseOrient(DriveTrain m_Drive, Limelight m_l) {
        drive = m_Drive;
        // makes x and y the botpose
        lime = m_l;
        botpose = lime.getRobotPose();
        x = botpose.getX();
        y = botpose.getY();
    }

    public void execute() {
        System.out.println(x);
        System.out.println(y);
        if (Math.abs((STATION_FRONT_X + STATION_BACK_X / 2) - x) >= 0.5 ) {
            drive.drive(((STATION_FRONT_X + STATION_BACK_X / 2) - x) /15,0,0);
        }
    }

}
