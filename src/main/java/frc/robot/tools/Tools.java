package frc.robot.tools;

import edu.wpi.first.math.geometry.Pose2d;

public class Tools {
    public static double angleDegrees(Pose2d p1, Pose2d p2){
        return Math.toDegrees(Math.atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX()));
    }
}
