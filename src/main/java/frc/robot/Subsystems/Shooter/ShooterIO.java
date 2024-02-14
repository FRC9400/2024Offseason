
package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double handoffAppliedVolts = 0.0;
        public double handoffSpeedRPS = 0.0;

        public double appliedVolts = 0.0;
        public double[] shooterSpeedRPS = new double[] {};
        
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void updateTunableNumbers() {
    }

    public default void setHandoffVoltage(double volts){
    }

    public default void setVoltage(double volts) {
    }

    public default void setVelocity(double velocityRPS){    
    }

    public default void shooterConfiguration() {
    }
}
