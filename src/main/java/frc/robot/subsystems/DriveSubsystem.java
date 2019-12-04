/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static final CANSparkMax neo1 = new CANSparkMax(RobotMap.NEO_1, MotorType.kBrushless);
  public static final CANSparkMax neo2 = new CANSparkMax(RobotMap.NEO_2, MotorType.kBrushless);
  public static SpeedController leftSide;
  public static SpeedController rightSide;
  public static DifferentialDrive drive;

  public DriveSubsystem() {
    // set all NEOs to factory defaults
    neo1.restoreFactoryDefaults();
    neo2.restoreFactoryDefaults();
     // the old-school way using SpeedControllerGroup
    leftSide = new SpeedControllerGroup(neo1);
    rightSide = new SpeedControllerGroup(neo2);
    drive = new DifferentialDrive(leftSide, rightSide);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }
  
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

}