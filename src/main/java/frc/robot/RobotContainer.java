/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.elevatorCommand;
import frc.robot.commands.limelightTurretVisionCommand;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.turretSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final driveSubsystem m_driveSubsystem = new driveSubsystem();
  private final turretSubsystem m_turretSubsystem = new turretSubsystem();
  public static final elevatorSubsystem m_elevatorSubsystem = new elevatorSubsystem();
  public final elevatorCommand m_elevatorCommand = new elevatorCommand();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public static XboxController m_driveController = new XboxController(DriveConstants.k_driveController);
  public static XboxController m_operatorController = new XboxController(DriveConstants.k_operatorController);
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new driveCommand(m_driveSubsystem));
    m_turretSubsystem.setDefaultCommand(new limelightTurretVisionCommand(m_turretSubsystem));
    m_elevatorSubsystem.setDefaultCommand(new elevatorCommand());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  }*/
}
