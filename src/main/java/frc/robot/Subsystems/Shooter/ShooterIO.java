package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterAppliedVolts = 0.0;
        public double[] shooterSetpointsRPS = new double[] {};
        public double[] shooterSpeedRPS = new double[] {};
        public double[] shooterSetpointsMPS = new double[] {};
        public double[] shooterSpeedMPS = new double[] {};

        public double[] shooterCurrent = new double[] {};
        public double[] shooterTemp = new double[] {};

        public double armAppliedVolts = 0.0;

        public double armSetpointDeg = 0.0;
        public double armSetpointRot = 0.0;
        public double[] armPosDeg = new double[] {};
        public double[] armPosRot = new double[] {};

        public double[] armCurrent = new double[] {};
        public double[] armTemp = new double[] {};
        public double[] armRPS = new double[] {};
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void requestShooterVoltage(double voltage) {}

    public default void requestVelocity(double velocity, double ratio) {}

    public default void requestArmVoltage(double voltage) {}

    public default void requestSetpoint(double angleDegrees) {}

    public default void zeroShooterVelocity() {}

}
