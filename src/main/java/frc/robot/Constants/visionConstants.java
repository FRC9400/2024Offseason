package frc.robot.Constants;

import edu.wpi.first.math.geometry.Transform3d;

public final class visionConstants {
        public static final String CAMERA_NAME_1 = "Camera1";
        public static final String CAMERA_NAME_2 = "Camera2";
        public static final Transform3d ROBOT_TO_CAMERA1 = new Transform3d(
                /* x */ 0.2, /* y */ 0.0, /* z */ 0.5,
                /* rotation */ new edu.wpi.first.math.geometry.Rotation3d());
        public static final Transform3d ROBOT_TO_CAMERA2 = new Transform3d(
                /* x */ -0.2, /* y */ 0.0, /* z */ 0.5,
                /* rotation */ new edu.wpi.first.math.geometry.Rotation3d());
        }