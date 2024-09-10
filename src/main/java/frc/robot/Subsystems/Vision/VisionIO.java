package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public double timestamp = 0.0;
        public double latency = 0.0;

        public int numTargets = 0;
        public double[] targetYaws = new double[] {};
        public double[] targetDistances = new double[] {};

        public double timestampCamera1 = 0.0;
        public double latencyCamera1 = 0.0;
        public int numTargetsCamera1 = 0;
        public double[] targetYawsCamera1 = new double[] {};
        public double[] targetDistancesCamera1 = new double[] {};

        public double timestampCamera2 = 0.0;
        public double latencyCamera2 = 0.0;
        public int numTargetsCamera2 = 0;
        public double[] targetYawsCamera2 = new double[] {};
        public double[] targetDistancesCamera2 = new double[] {};

        public double[] chosenPoseTranslation = new double[3]; // X, Y, Z
        public double[] chosenPoseRotation = new double[3];    // Yaw, Pitch, Roll
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
