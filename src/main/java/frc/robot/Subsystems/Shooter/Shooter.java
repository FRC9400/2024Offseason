package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;


public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine shooterRoutine;
    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        shooterRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(6),null, (state) -> SignalLogger.writeString("state", state.toString())), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> shooterIO.setVoltage(volts.in(Volts)), null, this));
      }
    
    public Command shooterSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            shooterRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.shooterSpeedMPS[0] > 30),
                this.runOnce(() -> shooterIO.setVoltage(0)),
                Commands.waitSeconds(1),
            shooterRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.shooterSpeedMPS[0] < -30),
                this.runOnce(() -> shooterIO.setVoltage(0)),
                Commands.waitSeconds(1),  

            shooterRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.shooterSpeedMPS[0] > 15),
                this.runOnce(() -> shooterIO.setVoltage(0)),
                Commands.waitSeconds(1),  

            shooterRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.shooterSpeedMPS[0] < -15),
                this.runOnce(() -> shooterIO.setVoltage(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }
    
    public void shootVoltage(double volts){
        shooterIO.setVoltage(volts);
    }

    public void shootVelocity(){
        shooterIO.setVelocity();
    }

    public void zeroVelocity(){
        shooterIO.zeroVelocity();
    }

    public void shooterConfiguration(){
        shooterIO.shooterConfiguration();
    }

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
}
