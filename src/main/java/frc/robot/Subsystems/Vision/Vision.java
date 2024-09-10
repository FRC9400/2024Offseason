package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO visionIO) {
        this.visionIO = visionIO;
    }

    @Override
    public void periodic() {
        visionIO.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    public double[] getLatestPoseTranslation() {
        return inputs.chosenPoseTranslation;
    }

    public double[] getLatestPoseRotation() {
        return inputs.chosenPoseRotation;
    }
}
