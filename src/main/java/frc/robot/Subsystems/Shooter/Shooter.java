package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Volts;


public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private ShooterStates state = ShooterStates.IDLE;
    private double shooterVolts = 0;
    private double[] shooterVelocity = {0, 0}; //velocity, ratio
    private double armSetpointDeg = 0;
    private double armVolts = 0;

    public enum ShooterStates{
        IDLE,
        HOMING,
        SHOOT,
        VOLTAGE,
        ZEROPOSITION
    }

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        //Logger.recordOutput("ShooterState", state);

        switch(state){
            case IDLE:
                shooterIO.requestArmVoltage(0);
                shooterIO.requestShooterVoltage(0);
                break;
            case HOMING:
                shooterIO.requestArmVoltage(-1);
                shooterIO.requestShooterVoltage(0);
            case SHOOT:
                shooterIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterIO.requestSetpoint(armSetpointDeg);
                break;
            case VOLTAGE:
                shooterIO.requestArmVoltage(armVolts);
                shooterIO.requestShooterVoltage(shooterVolts);
                break;
            case ZEROPOSITION:
                shooterIO.zeroPosition();
                shooterIO.requestShooterVoltage(0);
        }
    }

    public void requestVoltage(double shooterVoltage, double armVoltage){
        this.shooterVolts = shooterVoltage;
        this.armVolts = armVoltage;
        setState(ShooterStates.VOLTAGE);
    }

    public void requestShoot(double velocity, double ratio, double armSetpointDeg){
        this.shooterVelocity[0] = velocity;
        this.shooterVelocity[1] = ratio;
        this.armSetpointDeg = armSetpointDeg;
        setState(ShooterStates.SHOOT);
    }

    public void requestIdle(){
        setState(ShooterStates.IDLE);
    }

    public void setState(ShooterStates nextState){
        this.state = nextState;
    }

    public ShooterStates getState(){
        return this.state;
    }

}
