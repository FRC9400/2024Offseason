package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempFahrenheit = 0.0;
        public double intakeSpeedRPS = 0.0;

    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

}