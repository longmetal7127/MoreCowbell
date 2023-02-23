// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Autonomous;
import frc.robot.commands.BFMActuate;
import frc.robot.commands.BrakeActuate;
import frc.robot.commands.ClawActuate;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.RotateArm;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pnuematics;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_drive = new DriveTrain();
  public final ArmTrain m_arm = new ArmTrain();
  
  public final Limelight m_limelight = new Limelight();
  public final NavX navx = new NavX();
  public final Pnuematics pnuematics = new Pnuematics();
  public final Autonomous m_autocommand = new Autonomous(m_drive, m_limelight);

  public static Joystick joystick = new Joystick(Constants.joystickPort);
  public static XboxController xbox = new XboxController(Constants.xboxPort);

  private final JoystickDrive joystickDrive = new JoystickDrive(m_drive);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
    m_drive.setDefaultCommand(joystickDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //xbox.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    JoystickButton sideButton = new JoystickButton(xbox, XboxController.Button.kB.value);
    sideButton.whileTrue(new BFMActuate(pnuematics, kForward)); 
    sideButton.whileFalse(new BFMActuate(pnuematics, kReverse));

    JoystickButton triggerButton = new JoystickButton(xbox, XboxController.Button.kX.value);
    triggerButton.whileTrue(new ClawActuate(pnuematics, kForward));
    triggerButton.whileFalse(new ClawActuate(pnuematics, kReverse));

    JoystickButton up = new JoystickButton(xbox, XboxController.Button.kY.value);
    up.whileTrue(new RotateArm(m_arm, -20, xbox));
    JoystickButton down = new JoystickButton(xbox, XboxController.Button.kA.value);
    down.whileTrue(new RotateArm(m_arm,0, xbox));
    JoystickButton brakesUp = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    brakesUp.whileTrue(new BrakeActuate(pnuematics, kForward));
    JoystickButton brakesDown = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
    brakesDown.whileTrue(new BrakeActuate(pnuematics, kReverse));


    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autocommand;
  }
}
