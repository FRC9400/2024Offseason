package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private double[] indexerVoltage = {0.0, 0.0}; //intake, handoff
    private double[] ampRollerVoltage = {0.0, 0.0, 0.0}; //intake, handoff, outake

    private IndexerStates state = IndexerStates.IDLE;

    public enum IndexerStates{
        IDLE,
        INTAKE,
        OUTAKE,
        HANDOFF
    }

    
    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public void requestIntake(double indexerVoltage, double ampRollerVoltage){
        this.indexerVoltage[0] = indexerVoltage;
        this.ampRollerVoltage[0] = ampRollerVoltage;
        setState(IndexerStates.INTAKE);
    }

    public void requestHandoff(double indexerVoltage, double ampRollerVoltage){
        this.indexerVoltage[1] = indexerVoltage;
        this.ampRollerVoltage[1] = ampRollerVoltage;
        setState(IndexerStates.HANDOFF);
    }

    public void requestOutake(double ampRollerVoltage){
        this.ampRollerVoltage[2] = ampRollerVoltage;
        setState(IndexerStates.OUTAKE);
    }

    public void setState(IndexerStates nextState){
        this.state = nextState;
    }

    public IndexerStates getState(){
        return this.state;
    }

    @Override
    public void periodic(){
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        //Logger.recordOutput("IndexerState", state.toString());

        switch(state){
            case IDLE:
                indexerIO.setIndexerVoltage(0);
                indexerIO.setAmpRollerVoltage(0);
                break;
            case INTAKE:
                indexerIO.setIndexerVoltage(indexerVoltage[0]);
                indexerIO.setAmpRollerVoltage(ampRollerVoltage[0]);
                break;
            case OUTAKE:
                indexerIO.setAmpRollerVoltage(ampRollerVoltage[2]);
                break;
            case HANDOFF:
                indexerIO.setIndexerVoltage(indexerVoltage[1]);
                indexerIO.setAmpRollerVoltage(ampRollerVoltage[1]);
                break;
            default:
                break;
            }
    }
}