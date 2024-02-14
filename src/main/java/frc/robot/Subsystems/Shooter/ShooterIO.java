
package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double appliedVolts = 0.0;
        public double[] shooterSpeedRPS = new double[] {};
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};


    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void updateTunableNumbers() {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVelocity(double velocityRPS){    
    }

    public default void shooterConfiguration() {
    }
}
