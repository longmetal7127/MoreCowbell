package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class GoTo extends CommandBase {
    private DriveTrain drive;
    private Limelight lime;

    private double targetX;
    private double targetY;
    private double dX;
    private double dY;

    private double velocity;
    private double tolerance; // position tolerance

    public GoTo(DriveTrain m_drive, Limelight m_limelight, double targetX, double targetY, double velocity,
            double tolerance) {
        this.drive = m_drive;
        this.lime = m_limelight;

        this.targetX = targetX;
        this.targetY = targetY;
        this.velocity = velocity;
        this.tolerance = tolerance;
    }

    public void execute() {

        Pose3d botpose = lime.getSmoothRobotPose();
        Rotation3d rotation = botpose.getRotation();
        double x = botpose.getX();
        double y = botpose.getY();
        this.dY = targetY - y;
        this.dX = targetX - x;
        System.out.println("bot x: " + x);
        System.out.println("bot y: " + y);

        double joystickX = 0;
        double joystickY = 0;

        if (Math.abs(this.dX) > tolerance) {
            // velocity proportional to distance
            joystickX = this.dX / 10;

            // velocity constant X
            /*
             * if(dX > 0) {
             * joystickX = velocity;
             * } else if(dX < 0) {
             * joystickX = -velocity;
             * }
             */
        }

        if (Math.abs(this.dY) > tolerance) {
            // velocity proportional to distance
            joystickY = this.dY / 10;

            // velocity constant Y
            /*
             * if(dY > 0) {
             * joystickY = velocity;
             * } else if(dY < 0) {
             * joystickY = -velocity;
             * }
             */
        }

        if (joystickX != 0 || joystickY != 0) {
            System.out.println("joystickX: " + joystickX);
            System.out.println("joystickY: " + joystickY);

            drive.drive(joystickX, -joystickY, 0);
        }
    }

    public boolean isFinished() {
        return (Math.abs(this.dY) > tolerance && Math.abs(this.dX) > tolerance);
    }

}
