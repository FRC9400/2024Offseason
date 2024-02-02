package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    Pigeon2 pigeon;

    public GyroIOPigeon2(int pigeonID){
        pigeon = new Pigeon2(pigeonID);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.positionDegRaw = pigeon.getYaw().getValue();
        inputs.positionRad = Units.degreesToRadians(inputs.positionDegRaw);
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
