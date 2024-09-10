package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private int logCounter = 0;
    private static final int LOG_FREQUENCY = 10; //every 10 cycles

    public Vision(VisionIO visionIO) {
        this.visionIO = visionIO;
    }

    @Override
    public void periodic() {
        visionIO.updateInputs(inputs);
        if (logCounter++ >= LOG_FREQUENCY) {
            Logger.processInputs("Vision", inputs);
            logCounter = 0;
        }
    }

    public double[] getLatestPoseTranslation() {
        return inputs.chosenPoseTranslation;
    }

    public double[] getLatestPoseRotation() {
        return inputs.chosenPoseRotation;
    }
}
