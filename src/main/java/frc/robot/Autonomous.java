/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.AutoConstants.kRamseteB;
import static frc.robot.Constants.AutoConstants.kRamseteZeta;
import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.kPDriveVel;
import static frc.robot.Constants.DriveConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.DriveConstants.ksVolts;
import static frc.robot.Constants.DriveConstants.kvVoltSecondsPerMeter;
import java.util.List;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.driveSubsystem;

/**
 * Generate autonomous drive commands.
 * 
 * Based on the Tutorial: 
 *   https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/index.html
 * and tutorial code:
 *   https://github.com/wpilibsuite/allwpilib/tree/master/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecommand
 * clues on how to adapt this example to use SparkMax and NEOs
 *   https://www.chiefdelphi.com/t/wpilib-trajectory-tutorial-but-with-neos/372731
 * 
 */
public class Autonomous {

    private driveSubsystem m_robotDrive;
    private DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private TrajectoryConfig config;

    /**
     * Creates a new autonomous.
     * 
     * @param drive subsystem for the robot
     */
    public Autonomous(driveSubsystem drivetrain) {
        m_robotDrive = drivetrain;

        // Create a voltage constraint to ensure we don't accelerate too fast
        autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ksVolts,
                        kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, 10);

        // Create config for trajectory
        config = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
    }

    /**
     * create a new autonomous command using RamseteCommand class and trajectory following
     *
     * @return Autonomous Command class
     */
    public Command creatAutonomousCommand() {

        var initalTime = System.nanoTime();

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(kPDriveVel, 0.0, 0.0), new PIDController(kPDriveVel, 0.0, 0.0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        var dt = (System.nanoTime() - initalTime) / 1E6;
        System.out.println("RamseteCommand generation time: " + dt + "ms");

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

}
