package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {




  public Vision() {
    CameraServer.startAutomaticCapture();
    //CameraServer.startAutomaticCapture();

  }

  public void printRGBValues(){
    


  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
