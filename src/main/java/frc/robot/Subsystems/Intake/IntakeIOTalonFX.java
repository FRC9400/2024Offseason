package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.intakeConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intake = new TalonFX(canIDConstants.intakeMotor, "canivore");

    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Double> current= intake.getStatorCurrent();
    private final StatusSignal<Double> temp = intake.getDeviceTemp();
    private final StatusSignal<Double> RPS = intake.getRotorVelocity();

    private double setpointVolts;

    public IntakeIOTalonFX() {
        var intakeConfigs = new TalonFXConfiguration();
        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = intakeConstants.statorCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        intakeConfigs.MotorOutput.Inverted = intakeConstants.intakeInvert;
        
        intake.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS);

        intake.optimizeBusUtilization();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        inputs.appliedVolts = intakeRequest.Output;
        inputs.setPointVolts = this.setpointVolts;
        inputs.currentAmps = current.getValue();
        inputs.tempFahrenheit = temp.getValue();
        inputs.intakeSpeedRPS = RPS.getValue();
    }

    public void setOutput(double output) {
        this.setpointVolts = output;
        intake.setControl(intakeRequest.withOutput(output));
    }

}
