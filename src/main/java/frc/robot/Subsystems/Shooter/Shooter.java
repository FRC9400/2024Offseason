package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Volts;


public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine shooterRoutine;
    private ShooterStates state = ShooterStates.IDLE;
    private double shooterVolts = 0.0;
    private double[] shooterVelocity = {0, 0}; //vel, ratio
    
    public enum ShooterStates{
        IDLE,
        VOLTAGE,
        VELOCITY
    }

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("ShooterState", state);

        switch(state){
            case IDLE:
                shooterIO.setOutput(0);
                break;
            case VOLTAGE:
                shooterIO.setOutput(shooterVolts);
            case VELOCITY:
                shooterIO.setVelocity(shooterVelocity[1], shooterVelocity[2]);
                break;
            default:
                break;
        }

    }

    public void requestVoltage(double volts){
        shooterVolts = volts;
        setState(ShooterStates.VOLTAGE);
    }

    public void requestVelocity(double velocity, double ratio){
        this.shooterVelocity[0] = velocity;
        this.shooterVelocity[1] = ratio;
        setState(ShooterStates.VELOCITY);
    }

    public void setState(ShooterStates nextState){
        this.state = nextState;
    }


    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        shooterRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(4),null, (state) -> SignalLogger.writeString("state", state.toString())), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> shooterIO.setOutput(volts.in(Volts)), null, this));
      }

    public void shooterConfiguration(){
        shooterIO.shooterConfiguration();
    }
    
    public Command shooterSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            shooterRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.shooterSpeedMPS[0] > 15),
                this.runOnce(() -> shooterIO.setOutput(0)),
                Commands.waitSeconds(1),
            shooterRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.shooterSpeedMPS[0] < -15),
                this.runOnce(() -> shooterIO.setOutput(0)),
                Commands.waitSeconds(1),  

            shooterRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.shooterSpeedMPS[0] > 15),
                this.runOnce(() -> shooterIO.setOutput(0)),
                Commands.waitSeconds(1),  

            shooterRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.shooterSpeedMPS[0] < -15),
                this.runOnce(() -> shooterIO.setOutput(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }

}
