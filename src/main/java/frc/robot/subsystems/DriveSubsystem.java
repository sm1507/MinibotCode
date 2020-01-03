/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class driveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public static final TalonFX falcon1 = new TalonFX(DriveConstants.FALCON_1);
  public static final TalonFX falcon2 = new TalonFX(DriveConstants.FALCON_2);
  public static SpeedController leftSide;
  public static SpeedController rightSide;
  public static DifferentialDrive drive;

  public driveSubsystem() {
    drive = new DifferentialDrive(leftSide, rightSide);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
