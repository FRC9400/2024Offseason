package frc.robot.Subsystems.Handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.handoffConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HandoffIOTalonFX implements HandoffIO {
    private final TalonFX handoff = new TalonFX(canIDConstants.handoffMotor, "canivore");

    private VoltageOut handoffRequest = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Double> current = handoff.getStatorCurrent();
    private final StatusSignal<Double> temp = handoff.getDeviceTemp();
    private final StatusSignal<Double> RPS = handoff.getRotorVelocity();;

    public HandoffIOTalonFX() {
        var handoffConfigs = new TalonFXConfiguration()
        ;
        var handoffCurrentLimitConfigs = handoffConfigs.CurrentLimits;
        handoffCurrentLimitConfigs.StatorCurrentLimit = handoffConstants.statorCurrentLimit;
        handoffCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        handoffConfigs.MotorOutput.Inverted = handoffConstants.handoffInvert;
        
        handoff.getConfigurator().apply(handoffConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS);

        handoff.optimizeBusUtilization();
    }

    public void updateInputs(HandoffIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        inputs.appliedVolts = handoffRequest.Output;
        inputs.currentAmps = current.getValue();
        inputs.tempFahrenheit = temp.getValue();
        inputs.handoffSpeedRPS = RPS.getValue();
    }

    public void setHandoffVoltage(double output) {
        handoff.setControl(handoffRequest.withOutput(output));
    }

}
