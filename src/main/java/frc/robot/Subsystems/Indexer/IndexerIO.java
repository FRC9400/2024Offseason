package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs{
        public double indexerTemperature = 0.0;
        public double indexerAppliedVolts = 0.0;
        public double indexerCurrent = 0.0;
        public double indexerRPS = 0.0;
        public double indexerSetpointVolts = 0.0;

        public double ampRollerAppliedVolts = 0.0;
        public double ampRollerTemp = 0.0;
        public double ampRollerCurrent = 0.0;
        public double ampRollerRPS = 0.0;
        public double ampRollerSetpointVotls = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs){}

    public default void setIndexerVoltage(double voltage){}

    public default void setAmpRollerVoltage(double voltage) {}

}