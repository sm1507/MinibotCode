/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.colorSensor;
import frc.robot.subsystems.controlPanelMotors;
import frc.robot.subsystems.driveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final driveSubsystem m_driveSubsystem = new driveSubsystem();
  private final colorSensor m_colorSensor = new colorSensor();
  private final controlPanelMotors m_controlPanelMotors = new controlPanelMotors();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public static XboxController m_driveController = new XboxController(DriveConstants.k_driveController);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();  
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton ybutton = new JoystickButton(m_driveController, Button.kY.value);
    final JoystickButton xbutton = new JoystickButton(m_driveController, Button.kX.value);
  
    ybutton.whenPressed(() -> m_controlPanelMotors.setPosition(0), m_controlPanelMotors);
    xbutton.whenPressed(() -> m_controlPanelMotors.setPosition(19 * 4096), m_controlPanelMotors);
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
