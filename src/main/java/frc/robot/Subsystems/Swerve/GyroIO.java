package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double positionDegRaw = 0.0;
        public double velocityRadPerSec = 0.0;
        public double pitchDeg = 0.0;
        public double rollDeg = 0.0;
        public double pitchRad = 0.0;
        public double rollRad = 0.0;
        public double changeInPitch = 0.0;
        public double[] xyz_dps = new double[3];
        //public Rotation2d positionRotation2d = new Rotation2d();
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void reset() {}    

}
