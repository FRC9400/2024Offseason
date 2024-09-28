package frc.robot.Subsystems.Amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.indexerConstants;

public class AmpIOTalonFX implements AmpIO{
    
    private double ampRollerSetpointVolts;
    private VoltageOut ampRollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final TalonFX ampRoller = new TalonFX(canIDConstants.ampRollerMotor, "rio");
    private TalonFXConfiguration ampRollerConfigs = new TalonFXConfiguration();

    private final StatusSignal<Double> ampRollerCurrent= ampRoller.getStatorCurrent();
    private final StatusSignal<Double> ampRollerTemp = ampRoller.getDeviceTemp();
    private final StatusSignal<Double> ampRollerRPS = ampRoller.getRotorVelocity();

    public AmpIOTalonFX(){
        
        ampRollerConfigs.CurrentLimits.StatorCurrentLimit = indexerConstants.ampRollerCurrentLimit;
        ampRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        ampRollerConfigs.MotorOutput.Inverted = indexerConstants.ampRollerInvert;

        ampRoller.getConfigurator().apply(ampRollerConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll( 50,
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS);
            
        ampRoller.optimizeBusUtilization();
        ampRollerSetpointVolts = 0.0;
    }

    public void updateInputs(AmpIOInputs inputs){

        BaseStatusSignal.refreshAll(
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS);

        inputs.ampRollerAppliedVolts = ampRollerVoltageRequest.Output;
        inputs.ampRollerSetpointVotls = this.ampRollerSetpointVolts;
        inputs.ampRollerCurrent = ampRollerCurrent.getValue();
        inputs.ampRollerTemp = ampRollerTemp.getValue();
        inputs.ampRollerRPS = ampRollerRPS.getValue();
    }

    public void setAmpRollerVoltage(double voltage){
        this.ampRollerSetpointVolts = voltage;
        ampRoller.setControl(ampRollerVoltageRequest.withOutput(voltage));
    }
}
