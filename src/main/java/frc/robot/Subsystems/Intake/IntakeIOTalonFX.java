package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intake;

    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(true);

    public IntakeIOTalonFX() {
        intake = new TalonFX(16, "canivore");
        var intakeConfigs = new TalonFXConfiguration();
        //intakeConfigs.MotorOutput.Inverted = invert;

        intake.getConfigurator().apply(intakeConfigs);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = intakeRequest.Output;
        inputs.currentAmps = intake.getStatorCurrent().getValue();
        inputs.tempFahrenheit = intake.getDeviceTemp().getValue();
        inputs.intakeSpeedRPS = intake.getRotorVelocity().getValue();
    }

    public void setVoltage(double output) {
        intake.setControl(intakeRequest.withOutput(output));
    }

}
