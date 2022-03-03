// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.SwerveSubsystem;

public class GenerateTrajectory { 
    private GenerateTrajectory(){} 

    private static TrajectoryConfig baseConfig(SwerveSubsystem swerve){
        return new TrajectoryConfig(swerve.getMaxSpeed(), swerve.getMaxAcceleration())
                        .setKinematics(swerve.getKinematics());
    }

    public static Trajectory backupOneMeter(Pose2d currentPosition, SwerveSubsystem swerve) { 
        TrajectoryConfig config = baseConfig(swerve).setReversed(true);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            new Pose2d( 0, 0, new Rotation2d() ),
            List.of(),
            new Pose2d( -1, 0, new Rotation2d() ),
            config);
        return trajectory.transformBy( new Transform2d( trajectory.getInitialPose(), currentPosition )); 
    } 

    public static Trajectory driveTo(Pose2d currentPosition, Pose2d targetPosition, SwerveSubsystem swerve) {
        TrajectoryConfig config = baseConfig(swerve);
        Rotation2d facing = calculateAngle(currentPosition, targetPosition);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(currentPosition.getX(), currentPosition.getY(), facing),
            List.of(),
            new Pose2d(targetPosition.getX(), targetPosition.getY(), facing),
            config);        
        return trajectory;
    }

    public static Trajectory driveTrajectory(Pose2d currentPosition, Pose2d targetPosition, SwerveSubsystem swerve) {
        return TrajectoryGenerator.generateTrajectory(List.of(currentPosition, targetPosition), baseConfig(swerve));
    }

    public static Rotation2d calculateAngle(Pose2d p1, Pose2d p2)
    {
        double degrees = Math.toDegrees(Math.atan2(p2.getX() - p1.getX(), p2.getY() - p1.getY()));
        degrees = degrees + Math.ceil( -degrees / 360 ) * 360;
    
        return Rotation2d.fromDegrees(degrees);
    }
}