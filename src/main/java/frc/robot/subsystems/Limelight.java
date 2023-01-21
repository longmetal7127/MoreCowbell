// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
public NetworkTableEntry tv = table.getEntry("tv");
public NetworkTableEntry tx = table.getEntry("tx");
public NetworkTableEntry ty = table.getEntry("ty");
public NetworkTableEntry ta = table.getEntry("ta");
public NetworkTableEntry botpose = table.getEntry("botpose");
public NetworkTableEntry ledMode = table.getEntry("ledMode");

public double distanceInFeet;
public double distance;
public double theta;

  public Limelight() {
      //ledMode.setNumber(3);//Set led off
  }

  public void setLedOn(boolean on){
      if(on){
        ledMode.setNumber(3);
      }
      else{
        ledMode.setNumber(1);
      }
  }

  public double getXOffset(){
    return tx.getDouble(0.0);
  }

  public double getArea(){
    return ta.getDouble(0.0);
  }

  public double getYOffset(){
    return ty.getDouble(0.0);
  }

  public double getValidTarget(){

    return tv.getDouble(0.0);
  }

  public double getDistance(){
    theta = ((Constants.limelightAngle + getYOffset()) * Math.PI / 180);

    return (((Constants.targetHeight - Constants.limelightHeight)/Math.tan(theta))-Constants.limelightOffset);

  }

  public double calculateShooterSpeed(){
    double speed = 0.0;
    distance = getDistance();
    distanceInFeet = distance/12.0;

    double closeCalc = 0.85*0.85*(distanceInFeet + 0.2)*(distanceInFeet + 0.2);
    double middleCalc = 1.2*1.2*(distanceInFeet - 4.7)*(distanceInFeet - 4.7);
    double farCalc = 0.07*distance;

    if(distance < Constants.middleDistance && distance > Constants.closeDistance){
      speed = farCalc + 43.7;
    }
    else if(distance < Constants.farDistance && distance >= Constants.middleDistance){
      speed = farCalc + 41;
    }

    /*
    if(distance<Constants.closeDistance){
      speed = 0;//(closeCalc) + 40; // (4x/5)^2+40
    } else if(distance<Constants.middleDistance){
      speed = farCalc + 49.3;
        }
    else if(distance<=Constants.farDistance){
      speed = 0;//farCalc + 45.7;
    }
    */

    return speed/100;
  }

  public double calculateAutoShooterSpeed(){
    double speed = 0.0;
    distance = getDistance();
    distanceInFeet = distance/12.0;

    double closeCalc = 0.85*0.85*(distanceInFeet + 0.2)*(distanceInFeet + 0.2);
    double middleCalc = 1.2*1.2*(distanceInFeet - 4.7)*(distanceInFeet - 4.7);
    double farCalc = 0.07*distance;

    if(distance < Constants.middleDistance && distance > Constants.closeDistance){
      speed = farCalc + 44.3;
    }
    else if(distance < Constants.farDistance && distance >= Constants.middleDistance){
      speed = farCalc + 43.7;
    }


    return speed/100;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
