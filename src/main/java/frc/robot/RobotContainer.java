// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Vision;;

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
  public final NavX navx = new NavX();

  public final Limelight m_limelight = new Limelight(navx, m_drive);
  public final Vision m_vision = new Vision();

  public final Pneumatics m_pneumatics = new Pneumatics();
  public final Autonomous m_autocommand = new Autonomous(m_drive, m_limelight, m_pneumatics,navx, m_arm);

  public static Joystick joystick = new Joystick(Constants.joystickPort);
  public static XboxController xbox = new XboxController(Constants.xboxPort);

  private final JoystickDrive joystickDrive = new JoystickDrive(m_drive, joystick);
  private List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  private MecanumAutoBuilder autoBuilder = new MecanumAutoBuilder(
      m_limelight::getCurrentPose, // Pose2d supplier
      m_limelight::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_drive::setOutputSpeeds, // Output wheel speeds
      eventMap, // The event map with commands
      true, // Use alliance color
      m_drive // The drive subsystem. Used to properly set the requirements of path following commands
  );
  
  Command fullAuto = autoBuilder.fullAuto(pathGroup);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(joystickDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    return fullAuto;
  }
}
