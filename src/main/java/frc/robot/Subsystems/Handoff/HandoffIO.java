package frc.robot.Subsystems.Handoff;

import org.littletonrobotics.junction.AutoLog;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempFahrenheit = 0.0;
        public double handoffSpeedRPS = 0.0;

    }

    public default void updateInputs(HandoffIOInputs inputs) {
    }

    public default void setOutput(double volts) {
    }



}