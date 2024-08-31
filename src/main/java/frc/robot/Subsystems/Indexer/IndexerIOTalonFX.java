package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.indexerConstants;
import frc.robot.Subsystems.Indexer.IndexerIO.IndexerIOInputs;

public class IndexerIOTalonFX extends SubsystemBase{
    private final TalonFX indexer = new TalonFX(canIDConstants.indexerMotor);
    private final TalonFX ampRoller = new TalonFX(canIDConstants.ampRollerMotor);

    private TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration ampRollerConfigs = new TalonFXConfiguration();

    private VoltageOut indexerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private VoltageOut ampRollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    
    private final StatusSignal<Double> indexerCurrent= indexer.getStatorCurrent();
    private final StatusSignal<Double> indexerTemp = indexer.getDeviceTemp();
    private final StatusSignal<Double> indexerRPS = indexer.getRotorVelocity();

    private final StatusSignal<Double> ampRollerCurrent= ampRoller.getStatorCurrent();
    private final StatusSignal<Double> ampRollerTemp = indexer.getDeviceTemp();
    private final StatusSignal<Double> ampRollerRPS = indexer.getRotorVelocity();
    
    private double indexerSetpointVolts;
    private double ampRollerSetpointVolts;

    public IndexerIOTalonFX() {
        indexerConfigs.CurrentLimits.StatorCurrentLimit = indexerConstants.indexerCurrentLimit;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfigs.MotorOutput.Inverted = indexerConstants.indexerInvert;

        ampRollerConfigs.CurrentLimits.StatorCurrentLimit = indexerConstants.ampRollerCurrentLimit;
        ampRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        ampRollerConfigs.MotorOutput.Inverted = indexerConstants.ampRollerInvert;

        indexer.getConfigurator().apply(indexerConfigs);
        ampRoller.getConfigurator().apply(ampRollerConfigs);
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            indexerCurrent,
            indexerTemp,
            indexerRPS,
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS
            );

        indexer.optimizeBusUtilization();
        ampRoller.optimizeBusUtilization();

        indexerSetpointVolts = 0.0;
        ampRollerSetpointVolts = 0.0;
    }

    public void updateInputs(IndexerIOInputs inputs){
        BaseStatusSignal.refreshAll(
            indexerCurrent,
            indexerTemp,
            indexerRPS,
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS
        );
        inputs.indexerAppliedVolts = indexerVoltageRequest.Output;
        inputs.indexerSetpointVolts = this.indexerSetpointVolts;
        inputs.indexerCurrent = indexerCurrent.getValue();
        inputs.indexerTemperature = indexerTemp.getValue();
        inputs.indexerRPS = indexerRPS.getValue();

        inputs.ampRollerAppliedVolts = ampRollerVoltageRequest.Output;
        inputs.ampRollerSetpointVotls = this.ampRollerSetpointVolts;
        inputs.ampRollerCurrent = ampRollerCurrent.getValue();
        inputs.ampRollerTemp = ampRollerTemp.getValue();
        inputs.ampRollerRPS = ampRollerRPS.getValue();
    }
    
    public void setIndexerVoltage(double voltage){
        this.indexerSetpointVolts = voltage;
        indexer.setControl(indexerVoltageRequest.withOutput(voltage));
    }

    public void setAmpRollerVoltage(double voltage){
        this.ampRollerSetpointVolts = voltage;
        ampRoller.setControl(ampRollerVoltageRequest.withOutput(voltage));
    }
}
