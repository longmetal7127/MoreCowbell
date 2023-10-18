package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class FollowAprilTag extends CommandBase {
    private boolean isFinished = false;
    private DriveTrain m_drive;
    private Limelight m_limelight;
    private ArmTrain m_armtrain;
    public FollowAprilTag(DriveTrain m_drive, Limelight m_limelight, ArmTrain m_ArmTrain) {
        this.m_drive = m_drive;
        this.m_limelight = m_limelight;
        this.m_armtrain = m_ArmTrain;
    }

    double lastTA = 0;
    int state = 0;

    public void execute() {
        double tx = m_limelight.tx.getDouble(9999);
        double ta = m_limelight.ta.getDouble(9999);

        // double tv = m_limelight.tv.getDouble(9999);
        if (state == 0) {
            if (ta > 0) {
                lastTA = ta;
            }

            if (lastTA > 1.8) {
                m_drive.drive(0, 0, 0);
                state = 1;

            } else if (ta != 0) {
                m_drive.drive(0.1, 0, 0.02 * tx);
            }
        } else if (state == 1) {
            System.out.println(state);
            if (tx < 14) {
                System.out.println(tx);

                m_drive.drive(0, 0.4,0);

            } else {
                state = 2;
            }
        } else if (state ==2) {
            m_armtrain.rotate(-20);
            
        }

    }

    public boolean isFinished() {
        return isFinished;
    }
}
