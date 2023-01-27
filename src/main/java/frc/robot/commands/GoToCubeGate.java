package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;

public class GoToCubeGate {
    public static void goCube(int gatenum) {
        switch (gatenum){
            case 1:
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                    //goTo(x, y)
                }
                else {
                    //goTo(-x, y)
                }
                break;
            case 2:
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                //goTo(x, y)
                }
                else {
                //goTo(-x, y)
                }
                break;
            case 3:
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                    //goTo(x, y)
                }
                else {
                   //goTo(-x, y)
                }
                break;
        }
}

    public void goRight(int x) {
        //goTo(this.x, y-x); 
        //x is 22 inches; figure out what that is in the coordinate plane
    }

    public void goLeft(int x) {
        //goTo(this.x, y-x);
        //x is 22 inches; figure out what that is in the coordinate plane
    }
}
