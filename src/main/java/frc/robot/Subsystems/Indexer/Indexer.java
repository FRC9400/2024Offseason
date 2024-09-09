package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer{
    private final IndexerIO indexerIO;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerStates indexerState = IndexerStates.IDLE;
    private double[] indexerVoltage = {0,0}; // 0 = handoff, 1 = amp

    public enum IndexerStates{
        IDLE,
        HANDOFF,
        SHOOT,
        AMP
    }
    
    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

  
    public void Loop(){
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("IndexerState", this.indexerState);
        switch(indexerState){
            case IDLE:
                indexerIO.setIndexerVoltage(0);
                indexerIO.setAmpRollerVoltage(0);
                break;
            case HANDOFF:
                indexerIO.setIndexerVoltage(indexerVoltage[0]);
                indexerIO.setAmpRollerVoltage(indexerVoltage[1]);
                break;
            case SHOOT:
                indexerIO.setIndexerVoltage(0);
                indexerIO.setAmpRollerVoltage(indexerVoltage[1]);
            case AMP:
                indexerIO.setIndexerVoltage(0);
                indexerIO.setAmpRollerVoltage(indexerVoltage[1]);
                break;
            default:
                break;

        }
    }

    public void requestIdle(){
        setState(IndexerStates.IDLE);
    }

    public void requestHandoff(double indexerVoltage, double amppVoltage){
        this.indexerVoltage[0] = indexerVoltage;
        this.indexerVoltage[1] = amppVoltage;
        setState(IndexerStates.HANDOFF);
    }

    public void requestShoot(double ampVoltage){
        this.indexerVoltage[1] = ampVoltage;
        setState(IndexerStates.SHOOT);
    }

    public void requestAmp(double ampVoltage){
        this.indexerVoltage[1] = ampVoltage;
        setState(IndexerStates.AMP);
    }

    public void setState(IndexerStates nextState){
        this.indexerState = nextState;
    }

    public double getAmpCurrent(){
        return inputs.ampRollerCurrent;
    }

    public double getIndexerCurrent(){
        return inputs.indexerCurrent;
    }

    public IndexerStates getIndexerState(){
        return this.indexerState;
    }
}