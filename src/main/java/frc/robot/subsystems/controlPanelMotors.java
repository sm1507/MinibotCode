/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class controlPanelMotors extends SubsystemBase {
  /**
   * Creates a new controlPanelMotors.
   */
  private static final int kMotorPort = 0;
  private static final int kJoystickPort = 1;

  private SpeedController m_motor;
  private Joystick m_joystick;


  public controlPanelMotors() {
    m_motor = new WPI_TalonFX(kMotorPort);
    m_joystick = new Joystick(kJoystickPort);
  }


   public void setSpeed(double speed) {
    WPI_TalonFX.set(speed);
}
}
