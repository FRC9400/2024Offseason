package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    
    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;

    }

    @Override
    public void periodic(){
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
}