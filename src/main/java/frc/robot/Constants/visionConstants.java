package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class visionConstants {//literally everything in here is placeholders lol
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public static final Transform3d kRobotToCamShrek = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));//placeholders
    public static final Transform3d kRobotToCamDonkey = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));//placeholders
}
