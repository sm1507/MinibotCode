/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class elevatorcommand extends CommandBase {
  /**
   * Creates a new elevatorcommand.
   */
  public elevatorcommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Object elevatorSubsystem;
    elevatorSubsystem.elevatorWinch.setNeutralMode(NeutralMode.break);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Object elevatorSubsystem;
    elevatorSubsystem.elevatorWinch.set(ControlMode.PercentOutput, 1);


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
