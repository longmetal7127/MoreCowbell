package frc.robot.commands;

public class GoToCubeGate {
    public void goCube(int gatenum, String team) {
        switch (gatenum){
            case 1:
                if (team.equalsIgnoreCase("Blue")) {
                    //goTo(x, y)
                }
                else {
                    //goTo(-x, y)
                }
                break;
            case 2:
                if (team.equalsIgnoreCase("Blue")) {
                //goTo(x, y)
                }
                else {
                //goTo(-x, y)
                }
                break;
            case 3:
                if (team.equalsIgnoreCase("Blue")) {
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
