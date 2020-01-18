/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class driveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public static final CANSparkMax neo1 = new CANSparkMax(DriveConstants.NEO_1, MotorType.kBrushless);
  public static final CANSparkMax neo2 = new CANSparkMax(DriveConstants.NEO_2, MotorType.kBrushless);
  public static SpeedController leftSide;
  public static SpeedController rightSide;
<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/driveSubsystem.java
  public static DifferentialDrive drive;

  public driveSubsystem() {
    // set all NEOs to factory defaults
    neo1.restoreFactoryDefaults();
    neo2.restoreFactoryDefaults();
    leftSide = new SpeedControllerGroup(neo1);
    rightSide = new SpeedControllerGroup(neo2);
    drive = new DifferentialDrive(leftSide, rightSide);
=======
  //DifferentialDrive drive;

  public DriveSubsystem() {
    leftSide = new SpeedControllerGroup(falcon1);
    rightSide = new SpeedControllerGroup(falcon2);
    //drive = new DifferentialDrive(leftSide, rightSide);
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/DriveSubsystem.java
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void arcadeDrive(double xSpeed, double zRotation) {
    //drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    //drive.tankDrive(leftSpeed, rightSpeed);
  }
}
