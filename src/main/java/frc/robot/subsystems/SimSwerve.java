// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimSwerve extends SubsystemBase implements SwerveSubsystem{
    private long ticks = System.currentTimeMillis();
    private final Field2d field = new Field2d();

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics( new Translation2d[] {
            new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(-13)),    // Front Left
            new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(13)),     // Front Right
            new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)),    // Back Left
            new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13))      // Back Right
        });

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(KINEMATICS, Rotation2d.fromDegrees(0)); 
    private SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    public SimSwerve() {
        field.setRobotPose( getPose() );
        SmartDashboard.putData("Field", field);
    }
    
    public void drive(double xSpeed, double ySpeed, double zSpeed){ 
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        moduleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds) ;
    } 

    public void periodic() {
        double elapsed = (System.currentTimeMillis() - ticks) * 0.001;
        ticks = System.currentTimeMillis();

        ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(moduleStates);
        double finalAngle = odometry.getPoseMeters().getRotation().getRadians()
                            + (speeds.omegaRadiansPerSecond * elapsed) ;
        odometry.update(new Rotation2d(finalAngle), moduleStates);

        field.setRobotPose( getPose() );
        SmartDashboard.putData("Field", field);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters(); 
    }


    public void setPose(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        moduleStates = states;
    }


    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    public static final double maxVelocity       =  3.0 ; 
    public static final double maxAngularSpeed   = 180.0 ;

    public double getMaxSpeed() {
        return 3.0;
    }

    public double getMaxAcceleration() {
        return 1.0;
    }

    public double getMaxAngularSpeed() {
        return Math.PI/2.0;
    }

    public double getMaxAngularAcceleration() {
        return Math.PI/4.0;
    } 
}