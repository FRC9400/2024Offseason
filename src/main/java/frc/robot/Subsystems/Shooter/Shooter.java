package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
      }
    
    public void shootVoltage(double volts){
        shooterIO.setVoltage(volts);
    }

    public void shootVelocity(double velocityRPS){
        shooterIO.setVelocity(velocityRPS);
    }


    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
}
