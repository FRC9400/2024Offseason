package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    Pigeon2 pigeon;
    private final StatusSignal<Double> positionDegRaw;

    public GyroIOPigeon2(int pigeonID){
        pigeon = new Pigeon2(pigeonID, "canivore");
        positionDegRaw = pigeon.getYaw();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            positionDegRaw
        );

        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            positionDegRaw
        );
        inputs.connected = true;
        inputs.positionDegRaw = positionDegRaw.getValue();
        inputs.positionRad = Units.degreesToRadians(positionDegRaw.getValue());
        inputs.velocityRadPerSec = 0.0;
        inputs.pitchDeg = pigeon.getPitch().getValue();
        inputs.rollDeg = pigeon.getRoll().getValue();
        inputs.pitchRad = Units.degreesToRadians(pigeon.getPitch().getValue());
        inputs.rollRad = Units.degreesToRadians(inputs.rollDeg);
        //pigeon.getRawGyro(inputs.xyz_dps);
        //inputs.changeInPitch = filter.calculate(inputs.xyz_dps[1]);
    }


    @Override
    public void reset() {
        pigeon.setYaw(0.0);
    }

}
