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


public class ShooterArm{
    private final ShooterArmIO shooterArmIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ShooterArmStates shooterArmState = ShooterArmStates.IDLE;
    private double[] shooterVelocity = {0,0}; //left vel + ratio
    private double armAngle = 0;

    public enum ShooterArmStates{
        IDLE,
        SHOOT,
        AMP,
        PASS
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
            case SHOOT:
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterArmIO.requestPositionSetpoint(armAngle);
                break;
            case AMP:
                shooterArmIO.requestVelocity(0, 0);
                shooterArmIO.requestMotionMagicSetpoint(140);
                break;
            case PASS:
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterArmIO.requestPositionSetpoint(armAngle);
                break;
            default:
                break;

        }
    }

    public void requestIdle(){
        setState(shooterArmState.IDLE);
    }

    public void requestShoot(double velocityRPS, double ratio, double armAngle){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        this.armAngle = armAngle;
        setState(shooterArmState.SHOOT);
    }

    public void requestAmp(){
        setState(shooterArmState.AMP);
    }

    public void requestPass(double velocityRPS, double ratio, double armAngle){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        this.armAngle = armAngle;
        setState(shooterArmState.PASS);
    }

    public void setState(ShooterArmStates nextState){
        this.shooterArmState = nextState;
    }

    
}
