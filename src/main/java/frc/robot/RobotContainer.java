/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autonomous;
import frc.robot.commands.driveCommand;
import frc.robot.subsystems.driveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
  /* 
   * Subsystems
   */   
  private static final driveSubsystem m_driveSubsystem = new driveSubsystem();
  public static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  /* 
   * Commands
   */  
  // private final Command m_driveCommand = new driveCommand(m_driveSubsystem);
  private final Command m_autoCommand = new autonomous();

  public static XboxController m_driveController = new XboxController(OIConstants.kDriverController);



  public RobotContainer() {

    // default command is arcade drive command
    m_driveSubsystem.setDefaultCommand(new driveCommand(m_driveSubsystem));

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
    // TODO: define button commands

    // Example button
    // TODO: convert 4 to XboxController.kY when WPIlib is updated
    final JoystickButton ybutton = new JoystickButton(m_driveController, 4);

    // ybutton.whenPressed(new fooCommand(m_fooSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
