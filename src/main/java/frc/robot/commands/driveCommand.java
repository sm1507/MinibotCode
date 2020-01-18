/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.driveSubsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class driveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final driveSubsystem m_driveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public driveCommand(driveSubsystem subsystem) {
    m_driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    System.out.println("Configuring Motion Magic");
    System.out.println("Setting Motor Output");
    DriveSubsystem.falcon2.follow(DriveSubsystem.falcon1);
    DriveSubsystem.falcon1.configNominalOutputForward(0, Constants.kTimeoutMs);
		DriveSubsystem.falcon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
    DriveSubsystem.falcon1.configPeakOutputForward(1, Constants.kTimeoutMs);
    DriveSubsystem.falcon1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    System.out.println("Setting PIDF Values");
    DriveSubsystem.falcon1.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		DriveSubsystem.falcon1.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		DriveSubsystem.falcon1.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		DriveSubsystem.falcon1.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    DriveSubsystem.falcon1.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    System.out.println("Configuring Motion Velocity/Acceleration");
    DriveSubsystem.falcon1.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    DriveSubsystem.falcon1.configMotionAcceleration(6000, Constants.kTimeoutMs);
    System.out.println("Motion Magic Configuration Complete!");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream:src/main/java/frc/robot/commands/driveCommand.java
    m_driveSubsystem.arcadeDrive(RobotContainer.m_driveController.getY(Hand.kLeft), RobotContainer.m_driveController.getX(Hand.kRight));
=======
    //m_driveSubsystem.arcadeDrive(RobotContainer.m_driveController.getY(Hand.kLeft), RobotContainer.m_driveController.getX(Hand.kRight));
    double targetPos =  RobotContainer.m_driveController.getY(Hand.kLeft) * 4096 * 10.0;
    m_driveSubsystem.falcon1.set(ControlMode.MotionMagic, targetPos);
>>>>>>> Stashed changes:src/main/java/frc/robot/commands/DriveCommand.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
