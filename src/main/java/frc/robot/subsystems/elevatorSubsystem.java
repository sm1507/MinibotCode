/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class elevatorSubsystem extends SubsystemBase {

  public double P;
  public double I;
  public double D;
  public double F;

  private final WPI_TalonFX elevator1 = new WPI_TalonFX(elevatorConstants.elevator1);
  private final WPI_TalonFX elevator2 = new WPI_TalonFX(elevatorConstants.elevator2);
  public final static WPI_TalonFX elevatorWinch = new WPI_TalonFX(elevatorConstants.elevatorWinch);

  /**
   * Creates a new Climber.
   */
  public elevatorSubsystem() {
    elevator1.configFactoryDefault();
    elevator2.configFactoryDefault();
    elevator2.follow(elevator1);
    elevator2.setInverted(false);
    elevator1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
        elevatorConstants.elevatorSlotIdx,
        elevatorConstants.elevatorPivotTimeout);
    elevator1.setSensorPhase(true);
    elevator1.configNominalOutputForward(0, elevatorConstants.elevatorPivotTimeout);
    elevator1.configNominalOutputReverse(0, elevatorConstants.elevatorPivotTimeout);
    elevator1.configPeakOutputForward(0.25, elevatorConstants.elevatorPivotTimeout);
    elevator1.configPeakOutputReverse(-0.25, elevatorConstants.elevatorPivotTimeout);
    elevator1.setNeutralMode(NeutralMode.Brake);
    elevator2.setNeutralMode(NeutralMode.Brake);

    elevator1.configAllowableClosedloopError(elevatorConstants.elevatorSlotIdx, 0, elevatorConstants.elevatorPivotTimeout);
    P = 0.1;
    I = 0;
    D = 0;
    F = 0;
    
    setElevatorPID(P, I, D, F);
  }

  public void setElevatorPosition(double desiredPosition) {

   
    elevator1.set(ControlMode.Position, desiredPosition);
  }

  @Override
  public void periodic() {
     SmartDashboard.putNumber("Elevator_1_Pos", elevator1.getSelectedSensorPosition());
     SmartDashboard.putNumber("Evevator_2_Pos", elevator2.getSelectedSensorPosition());
  }

  public void setElevatorPID(double P, double I, double D, double F)

  {
    elevator1.config_kP(elevatorConstants.elevatorSlotIdx, P, elevatorConstants.elevatorPivotTimeout);
    elevator1.config_kP(elevatorConstants.elevatorSlotIdx, I, elevatorConstants.elevatorPivotTimeout);
    elevator1.config_kP(elevatorConstants.elevatorSlotIdx, D, elevatorConstants.elevatorPivotTimeout);
    elevator1.config_kP(elevatorConstants.elevatorSlotIdx, F, elevatorConstants.elevatorPivotTimeout);
  }

}
