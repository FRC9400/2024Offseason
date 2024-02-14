package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intake;
   
    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(true);

    public IntakeIOTalonFX(int motorID) {
        intake = new TalonFX(motorID);
        var intakeConfigs = new TalonFXConfiguration();

        intakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intake.getConfigurator().apply(intakeConfigs);
    }

    public void updateInputs(IntakeIOInputs intakeInputs) {
        intakeInputs.intakeAppliedVolts = intakeRequest.Output;
        intakeInputs.intakeCurrentAmps = intake.getStatorCurrent().getValue();
        intakeInputs.intakeTempFahrenheit = intake.getDeviceTemp().getValue();
        intakeInputs.intakeSpeedRPS = intake.getRotorVelocity().getValue();
    }

    public void setIntakeVoltage(double volts) {
        intake.setControl(intakeRequest.withOutput(volts));
    }
}
