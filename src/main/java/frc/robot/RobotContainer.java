// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.HashMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
  public final NavX m_navx = new NavX();

  public final DriveTrain m_drive = new DriveTrain(m_navx);

  public final ArmTrain m_arm = new ArmTrain();
  public final GripperTrain m_gripper = new GripperTrain();

  public final Limelight m_limelight = new Limelight(m_navx, m_drive);
  //public final Camera m_camera = new Camera();

  public final Pneumatics m_pneumatics = new Pneumatics();
  // public final Autonomous m_autocommand = new Autonomous(m_drive, m_limelight,
  // m_pneumatics,navx, m_arm);

  public static Joystick joystick = new Joystick(Constants.joystickPort);
  public static XboxController xbox = new XboxController(Constants.xboxPort);

  private final JoystickDrive joystickDrive = new JoystickDrive(m_drive);

  private HashMap<String, Command> globalEventMap = new HashMap<>();

  // Create the AutoBuilder. This only needs to be created once when robot code
  // starts, not every time you want to create an auto command. A good place to
  // put this is in RobotContainer along with your subsystems.
  private MecanumAutoBuilder autoBuilder = new MecanumAutoBuilder(
      m_limelight::getCurrentPose, // Pose2d supplier
      m_limelight::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      new PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                       // PID controllers)
      new PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                       // controller)
      m_drive::setOutputSpeeds, // Output wheel speeds
      globalEventMap, // The event map with commands
      true, // Use alliance color
      m_drive, m_limelight // The drive subsystem. Used to properly set the requirements of path following
                           // commands
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
    JoystickButton brakes = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    brakes.onTrue(new BrakeActuate(m_pneumatics, kForward));
    brakes.toggleOnFalse(new BrakeActuate(m_pneumatics, kReverse));

    // One Eighty
    JoystickButton oneEightyLeft = new JoystickButton(joystick, 5);
    oneEightyLeft.onTrue(new Turn(m_drive, m_navx, -180));
    JoystickButton oneEightyRight = new JoystickButton(joystick, 6);
    oneEightyRight.onTrue(new Turn(m_drive, m_navx, 180));

    // Nudge
    JoystickButton turnLeft = new JoystickButton(joystick, 3);
    turnLeft.onTrue(new Turn(m_drive, m_navx, -5));
    JoystickButton turnRight = new JoystickButton(joystick, 4);
    turnRight.onTrue(new Turn(m_drive, m_navx, 5));

    // Intake/Eject
    JoystickButton gripperMotors = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
    gripperMotors.whileTrue(new SetGripperSpeed(m_gripper, Constants.GRIPPER_BACKWARD_SPEED));
    gripperMotors.whileFalse(new SetGripperSpeed(m_gripper, Constants.GRIPPER_FORWARD_SPEED));

    // right
    POVButton a = new POVButton(xbox, 90);
    a.onTrue(new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[3]));

    // up
    POVButton b = new POVButton(xbox, 0);
    b.onTrue(new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[2]));

    // left
    POVButton c = new POVButton(xbox, 270);
    c.onTrue(new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[1]));

    // bottom
    POVButton d = new POVButton(xbox, 180);
    d.onTrue(new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[0]));

    // Actuate claw
    JoystickButton triggerButton = new JoystickButton(xbox, XboxController.Button.kX.value);
    triggerButton.toggleOnTrue(new ClawActuate(m_pneumatics, kForward, false));
    triggerButton.onFalse(new ClawActuate(m_pneumatics, kReverse, true));

    //JoystickButton setLights = new JoystickButton(xbox, XboxController.Button.kB.value);
    //setLights.onTrue(new LightSetColor(m_lights, 255, 255, 0));
    //setLights.onFalse(new LightSetColor(m_lights, 191, 64, 191));
  }

  private void configureEventMap() {
    for (int i = 0; i < Constants.ARM_HEIGHTS.length; i++) {
      globalEventMap.put("arm" + i, new ArmToPosition(m_arm, Constants.ARM_HEIGHTS[i]));
    }

    globalEventMap.put("openClaw", new ClawActuate(m_pneumatics, kForward, true));
    globalEventMap.put("closeClaw", new ClawActuate(m_pneumatics, kReverse, true));

    globalEventMap.put("eject", new SetGripperSpeed(m_gripper, Constants.GRIPPER_BACKWARD_SPEED));
    globalEventMap.put("intake", new SetGripperSpeed(m_gripper, Constants.GRIPPER_FORWARD_SPEED));

    globalEventMap.put("brakesOn", new BrakeActuate(m_pneumatics, kForward));
    globalEventMap.put("brakesOff", new BrakeActuate(m_pneumatics, kReverse));
  }

  public Command getAutonomousCommand(String location) {
    List<PathPlannerTrajectory> pathGroup = null;
    PathConstraints pc = new PathConstraints(Constants.PATH_MAX_VELOCITY, Constants.PATH_MAX_ACCELERATION);

    switch (location) {
      case "Auto 1":
        pathGroup = PathPlanner.loadPathGroup("auto1", pc);
        break;
      case "Auto 2":
        pathGroup = PathPlanner.loadPathGroup("auto2", pc);
        break;
      case "Auto 3": // currently identical to auto 1
        pathGroup = PathPlanner.loadPathGroup("auto1", pc);
        break;
      case "Do Nothing":
        return null;
      // pathGroup = PathPlanner.loadPathGroup("doNothing", new PathConstraints(0,
      // 0));
    }

    return autoBuilder.fullAuto(pathGroup);
  }
}
