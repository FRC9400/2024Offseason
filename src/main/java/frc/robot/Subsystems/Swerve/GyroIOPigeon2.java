package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    Pigeon2 pigeon;
    private final StatusSignal<Double> positionDegRaw;
    private final StatusSignal<Double> pitchDeg;
    private final StatusSignal<Double> rollDeg;

    public GyroIOPigeon2(int pigeonID){
        pigeon = new Pigeon2(pigeonID, "canivore");
        positionDegRaw = pigeon.getYaw();
        pitchDeg = pigeon.getPitch();
        rollDeg = pigeon.getRoll();


        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            positionDegRaw,
            pitchDeg,
            rollDeg
        );

        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            positionDegRaw,
            pitchDeg,
            rollDeg
        );
        inputs.connected = true;
        inputs.positionDegRaw = positionDegRaw.getValue();
        inputs.positionRad = Units.degreesToRadians(positionDegRaw.getValue());
        inputs.velocityRadPerSec = 0.0;
        inputs.pitchDeg = pitchDeg.getValue();
        inputs.rollDeg = rollDeg.getValue();
        inputs.pitchRad = Units.degreesToRadians(pitchDeg.getValue());
        inputs.rollRad = Units.degreesToRadians(rollDeg.getValue());
        //pigeon.getRawGyro(inputs.xyz_dps);
        //inputs.changeInPitch = filter.calculate(inputs.xyz_dps[1]);
    }


    @Override
    public void reset() {
        pigeon.setYaw(0.0);
    }

    public void setPosition(double yawDegrees){
        pigeon.setYaw(yawDegrees);
    }

}
