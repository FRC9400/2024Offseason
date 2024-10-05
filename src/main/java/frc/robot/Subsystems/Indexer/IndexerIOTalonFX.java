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

public class IndexerIOTalonFX implements IndexerIO{
    private final TalonFX indexer = new TalonFX(canIDConstants.indexerMotor, "canivore");

    private TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();

    private VoltageOut indexerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    
    private final StatusSignal<Double> indexerCurrent= indexer.getStatorCurrent();
    private final StatusSignal<Double> indexerTemp = indexer.getDeviceTemp();
    private final StatusSignal<Double> indexerRPS = indexer.getRotorVelocity();
    
    private double indexerSetpointVolts;

    public IndexerIOTalonFX() {
        indexerConfigs.CurrentLimits.StatorCurrentLimit = indexerConstants.indexerCurrentLimit;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfigs.MotorOutput.Inverted = indexerConstants.indexerInvert;


        indexer.getConfigurator().apply(indexerConfigs);
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            indexerCurrent,
            indexerTemp,
            indexerRPS
            );

        indexer.optimizeBusUtilization();

        indexerSetpointVolts = 0.0;
    }

    public void updateInputs(IndexerIOInputs inputs){
        BaseStatusSignal.refreshAll(
            indexerCurrent,
            indexerTemp,
            indexerRPS
        );
        inputs.indexerAppliedVolts = indexerVoltageRequest.Output;
        inputs.indexerSetpointVolts = this.indexerSetpointVolts;
        inputs.indexerCurrent = indexerCurrent.getValue();
        inputs.indexerTemperature = indexerTemp.getValue();
        inputs.indexerRPS = indexerRPS.getValue();
    }

    public void setIndexerVoltage(double voltage){
        indexer.setControl(indexerVoltageRequest.withOutput(voltage));
    }
}
