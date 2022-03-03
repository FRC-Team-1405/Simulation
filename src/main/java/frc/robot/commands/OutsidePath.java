package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class OutsidePath extends SequentialCommandGroup {

    private static final Pose2d startPose = new Pose2d(6.5, 5, Rotation2d.fromDegrees(-30));
    private static final Pose2d target_1  = new Pose2d(Units.inchesToMeters(-129.0 + 324), Units.inchesToMeters( 82.0 + 162), Rotation2d.fromDegrees( 145.0));
    private static final Pose2d target_2  = new Pose2d(Units.inchesToMeters(-125.0 + 324), Units.inchesToMeters(-88.0 + 162), Rotation2d.fromDegrees(  45.0));
    private static final Pose2d waypoint  = new Pose2d(Units.inchesToMeters(-200.0 + 324), Units.inchesToMeters(-88.0 + 162), Rotation2d.fromDegrees(   0.0));

    public OutsidePath(SwerveSubsystem swerve) {
        addRequirements(swerve);

        addCommands( new TurnToAngle( startPose.getRotation().getDegrees()+180, swerve),
                     new RunPath( List.of( target_1, 
                                           waypoint,
                                           target_2, 
                                           startPose), 
                                           swerve) );

    }
}
