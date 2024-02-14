package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double handoffVoltage = 0;
    private double shootVoltage = 0;
    private double shootVelocity = 0;

    private boolean requestIdle = true;
    private boolean requestShootVelocity = false;
    private boolean requestShootVoltage = false;
    private ShooterStates systemState = ShooterStates.IDLE;
    private ShooterStates nextSystemState = systemState;

    public enum ShooterStates {
        IDLE,
        SHOOT_VELOCITY,
        SHOOT_VOLTAGE
    }


    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
      }


        @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("ShooterState", systemState.toString());
        Logger.recordOutput("ShooterVoltageSetpoint", shootVoltage);
        Logger.recordOutput("ShooterVelocitySetpoint", shootVelocity);
        Logger.recordOutput("HandoffVoltageSetpoint", handoffVoltage);

        if(systemState == ShooterStates.IDLE){
            shooterIO.setVoltage(0);
            shooterIO.setHandoffVoltage(0);

            if(requestShootVoltage){
                nextSystemState = ShooterStates.SHOOT_VOLTAGE;
            }
            else if(requestShootVelocity){
                nextSystemState = ShooterStates.SHOOT_VELOCITY;
            }
        }
        else if (systemState == ShooterStates.SHOOT_VOLTAGE){
            shooterIO.setVoltage(shootVoltage);
            shooterIO.setHandoffVoltage(handoffVoltage);

            if(requestIdle){
                nextSystemState = ShooterStates.IDLE;
            }
        }
        else if (systemState == ShooterStates.SHOOT_VELOCITY){
            shooterIO.setVoltage(shootVelocity);
            shooterIO.setHandoffVoltage(handoffVoltage);

            if(requestIdle){
                nextSystemState = ShooterStates.IDLE;
            }
        }
        
        if(systemState != nextSystemState){
            systemState = nextSystemState;
        }
    }

    public void requestIdle(){
        requestIdle = true;
        requestShootVelocity = false;
        requestShootVoltage = false;
    }

    public void requestShootVelocity(double velocity, double handoffVoltage){
        requestIdle = false;
        requestShootVelocity = true;
        requestShootVoltage = false;

        this.shootVelocity = velocity;
        this.handoffVoltage = handoffVoltage;
    }

    public void requestVoltage(double voltage, double handoffVoltage){
        requestIdle = false;
        requestShootVelocity = false;
        requestShootVoltage = true;

        this.shootVoltage = voltage;
        this.handoffVoltage = handoffVoltage;
    }


}
