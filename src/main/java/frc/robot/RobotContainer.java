// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.GripperTrain;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_drive = new DriveTrain();
  public final ArmTrain m_arm = new ArmTrain();
  public final GripperTrain m_gripper = new GripperTrain();

  public final NavX navx = new NavX();

  public final Limelight m_limelight = new Limelight(navx, m_drive);
  public final Vision m_vision = new Vision();

  public final Pneumatics m_pneumatics = new Pneumatics();
  //public final Autonomous m_autocommand = new Autonomous(m_drive, m_limelight, m_pneumatics,navx, m_arm);

  public static Joystick joystick = new Joystick(Constants.joystickPort);
  public static XboxController xbox = new XboxController(Constants.xboxPort);

  private final JoystickDrive joystickDrive = new JoystickDrive(m_drive);

  private HashMap<String, Command> globalEventMap = new HashMap<>();

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  private MecanumAutoBuilder autoBuilder = new MecanumAutoBuilder(
      m_limelight::getCurrentPose, // Pose2d supplier
      m_limelight::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      new PIDConstants(0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_drive::setOutputSpeeds, // Output wheel speeds
      globalEventMap, // The event map with commands
      true, // Use alliance color
      m_drive, m_limelight // The drive subsystem. Used to properly set the requirements of path following commands
  );
  
  public RobotContainer() {
   // PathPlannerServer.startServer(5811);
    // Configure the trigger bindings
    configureBindings();

    // Configure the auto event map
    configureEventMap();

    // Default drive command is joystick
    m_drive.setDefaultCommand(joystickDrive);
  }

  private void configureBindings() {
    // Actuate claw
    xbox.setRumble(RumbleType.kBothRumble, 1);
    JoystickButton triggerButton = new JoystickButton(xbox, XboxController.Button.kX.value);
    triggerButton.whileTrue(new ClawActuate(m_pneumatics, kForward));
    triggerButton.whileFalse(new ClawActuate(m_pneumatics, kReverse));

    // Up arm
    JoystickButton up = new JoystickButton(xbox, XboxController.Button.kY.value);
    up.onTrue(new ArmCycle(m_arm, true));

    // Down arm
    JoystickButton down = new JoystickButton(xbox, XboxController.Button.kA.value);
    down.onTrue(new ArmCycle(m_arm, false));
    
    // Small adjustments on arm
    JoystickButton fineUp = new JoystickButton(xbox, XboxController.Button.kStart.value);
    fineUp.onTrue(new FineArmMove(m_arm, true));
    JoystickButton fineDown = new JoystickButton(xbox, XboxController.Button.kBack.value);
    fineDown.onTrue(new FineArmMove(m_arm, false));

    // Brakes
    JoystickButton brakesUp = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    brakesUp.whileTrue(new BrakeActuate(m_pneumatics, kForward));
    
    // Brakes
    JoystickButton brakesDown = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
    brakesDown.whileTrue(new BrakeActuate(m_pneumatics, kReverse));

    // Zero arm
    JoystickButton zero = new JoystickButton(xbox, XboxController.Button.kLeftStick.value);
    zero.whileTrue(new ZeroArm(m_arm));
    
    // One Eighty
    JoystickButton oneEightyLeft = new JoystickButton(joystick, 5);
    oneEightyLeft.onTrue(new Turn(m_drive, navx,-180));
    JoystickButton oneEightyRight = new JoystickButton(joystick, 6);
    oneEightyRight.onTrue(new Turn(m_drive, navx,180));

    // Nudge
    JoystickButton turnLeft = new JoystickButton(joystick, 3);
    turnLeft.onTrue(new Turn(m_drive,navx, -5));
    JoystickButton turnRight = new JoystickButton(joystick, 4);
    turnRight.onTrue(new Turn(m_drive,navx, 5));

    JoystickButton gripperMotors = new JoystickButton(xbox, XboxController.Button.kB.value);
    gripperMotors.whileTrue(new SetGripperSpeed(m_gripper, 0.2));
    gripperMotors.whileFalse(new SetGripperSpeed(m_gripper, 0));


  }

  private void configureEventMap() {
    globalEventMap.put("arm0", new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[0]));
    globalEventMap.put("arm1", new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[1]));
    globalEventMap.put("arm2", new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[2]));
    globalEventMap.put("arm3", new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[3]));
    globalEventMap.put("arm4", new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[4]));

    globalEventMap.put("openClaw", new ClawActuate(m_pneumatics, kForward));
    globalEventMap.put("closeClaw", new ClawActuate(m_pneumatics, kReverse));

    globalEventMap.put("brakesOn", new BrakeActuate(m_pneumatics, kForward));
    globalEventMap.put("brakesOff", new BrakeActuate(m_pneumatics, kReverse));
  }

  public Command getAutonomousCommand(String location) {
    System.out.print(location);
    List<PathPlannerTrajectory> pathGroup = null;
    PathConstraints pc = new PathConstraints(Constants.PATH_MAX_VELOCITY, Constants.PATH_MAX_ACCELERATION);

    DriverStation.Alliance alliance = DriverStation.getAlliance();
    //int location = DriverStation.getLocation();

    //System.out.println("Team: " + alliance + " " + location);
    
    switch(location) {
      case "Auto 1":
      pathGroup = PathPlanner.loadPathGroup("auto1", pc);
      break;
      case "Auto 2":
      pathGroup = PathPlanner.loadPathGroup("auto2", pc);
      break;
      case "Auto 3": //currently identical to auto 1
      pathGroup = PathPlanner.loadPathGroup("auto1", pc);
      break;
      case "Do Nothing":
        return null;
      //pathGroup = PathPlanner.loadPathGroup("doNothing", new PathConstraints(0, 0));

    }

    return autoBuilder.fullAuto(pathGroup);
  }
}
