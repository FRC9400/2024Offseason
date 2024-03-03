package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeTempFahrenheit = 0.0;
        public double intakeSpeedRPS = 0.0;

    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakeVoltage(double volts) {
    }



}