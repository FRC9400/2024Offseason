package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class visionConstants {//literally everything in here is placeholders lol
    public static final String[] name = {"Shrek", "Donkey"};

    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public static final Transform3d[] kRobotToCam = {new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)),
        new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0))};//placeholders

    public static final Matrix<N3,N1> kSingleTagStdDevs = VecBuilder.fill(0,0,0);//placeholder
    public static final Matrix<N3,N1> kMultiTagStdDevs = VecBuilder.fill(0,0,0);//more placeholder
}
