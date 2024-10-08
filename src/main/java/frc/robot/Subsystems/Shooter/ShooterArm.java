package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Volts;


public class ShooterArm{
    private final ShooterArmIO shooterArmIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ShooterArmStates shooterArmState = ShooterArmStates.IDLE;
    private double[] shooterVelocity = {0,0}; //left vel + ratio
    private double armAngleDegrees = 0;

    public enum ShooterArmStates{
        IDLE,
        ZERO,
        HANDOFF,
        SHOOT,
        AMP,
        PASS,
        VOLTAGE
    }

    public ShooterArm(ShooterArmIO shooterArmIO) {
        this.shooterArmIO = shooterArmIO;
    }

    
    public void Loop(){
        shooterArmIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("ShooterArmState", this.shooterArmState);
        switch(shooterArmState){
            case IDLE:
                shooterArmIO.requestArmVoltage(0);
                shooterArmIO.requestShooterVoltage(0);
                break;
            case ZERO:
                shooterArmIO.requestMotionMagicSetpoint(armAngleDegrees);
                shooterArmIO.requestShooterVoltage(0);
                break;
            case HANDOFF:
                 shooterArmIO.requestMotionMagicSetpoint(0);
                 shooterArmIO.requestShooterVoltage(0);
                 break;
            case SHOOT:
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterArmIO.requestMotionMagicSetpoint(armAngleDegrees);
                break;
            case AMP:
                shooterArmIO.requestVelocity(0, 0);
                shooterArmIO.requestMotionMagicSetpoint(130);
                break;
            case PASS:
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterArmIO.requestPositionSetpoint(armAngleDegrees);
                break;
            case VOLTAGE:
                shooterArmIO.requestShooterVoltage(4);
                shooterArmIO.requestArmVoltage(0);
            default:
                break;

        }
    }

    public void requestIdle(){
        setState(shooterArmState.IDLE);
    }

    public void requestShoot(double velocityRPS, double ratio, double armAngleDegrees){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        this.armAngleDegrees = armAngleDegrees;
        setState(shooterArmState.SHOOT);
    }

    public void requestAmp(){
        setState(shooterArmState.AMP);
    }

    public void requestZero(){
        setState(ShooterArmStates.ZERO);
    }

    public void requestHandoff(){
        setState(ShooterArmStates.HANDOFF);
    }
    
    public void requestPass(double velocityRPS, double ratio, double armAngleDeg){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        this.armAngleDegrees = armAngleDegrees;
        setState(shooterArmState.PASS);
    }

    public void requestVoltage(){
        setState(shooterArmState.VOLTAGE);
    }

    public void setState(ShooterArmStates nextState){
        this.shooterArmState = nextState;
    }

    public boolean atShooterSetpoint(){
        if(Math.abs(shooterVelocity[0] - inputs.shooterSpeedRPS[0]) < 0.1 && Math.abs((shooterVelocity[0] * shooterVelocity[1]) - inputs.shooterSpeedRPS[1]) < 0.1){
            return true;
        }
        return false;
    }

    public boolean atArmSetpoint(){
        if(Math.abs(armAngleDegrees - inputs.armPosDeg[0]) < 0.1){
            return true;
        }
        return false;
    }

    public double getArmDegrees(){
        return inputs.armPosDeg[0];
    }
    
}
