/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
public static final class DriveConstants {
    public static final int NEO_1  = 1;
    public static final int NEO_2 = 2;
    public static final int k_driveController = 0;
    public static final int k_operatorController = 1;
}

public static final class elevatorConstants{
    public static final int elevator1 = 3;
    public static final int elevator2 = 4;
    public static final int elevatorWinch = 5;
    public static final int elevatorPivotTimeout = 30;
    public static final int elevatorSlotIdx = 1;
    }
public static final class Manipulator {
    public static final int TURRET_DRIVE  = 1;
    public static final int kSoftMaxTurretAngle  = 1;
    public static final int kSoftMinTurretAngle  = 1;
    public static final int kTurretRotationsPerTick  = 1;

}
}