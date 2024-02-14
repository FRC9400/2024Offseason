
package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double intakeVoltage = 0.0;
    private double holdVoltage = 0.0;

    private IntakeStates systemState = IntakeStates.IDLE;
    private IntakeStates nextSystemState = systemState;
    private boolean requestIdle = true;
    private boolean requestIntake = false;
    private boolean requestHold = false;
    
    public enum IntakeStates{
        IDLE,
        INTAKE,
        HOLD
    }

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
      }
    
    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("IntakeState", systemState.toString());
        Logger.recordOutput("IntakeVoltageSetpoint", intakeVoltage);
        Logger.recordOutput("IntakeHoldVoltage", holdVoltage);

        if (systemState == IntakeStates.IDLE){
            intakeIO.setIntakeVoltage(0);

            if(requestIntake){
                nextSystemState = IntakeStates.INTAKE;
            }
            else if(requestHold){
                nextSystemState = IntakeStates.HOLD;
            }
        }
        else if (systemState == IntakeStates.INTAKE){
            intakeIO.setIntakeVoltage(intakeVoltage);

            if (requestIdle){
                nextSystemState = IntakeStates.IDLE;
            }
            else if(requestHold){
                nextSystemState = IntakeStates.HOLD;
            }
        }
        else if (systemState == IntakeStates.HOLD){
            intakeIO.setIntakeVoltage(intakeVoltage);

            if (requestIdle) {
                nextSystemState = IntakeStates.IDLE;
            }

        }

        if (systemState!=nextSystemState){
            systemState = nextSystemState;
        }
    }

    public void requestIdle(){
        requestIdle = true;
        requestIntake = false;
        requestHold = false;  
    }

    public void requestIntake(double voltage){
        requestIdle = false;
        requestIntake = true;
        requestHold = false;
        
        intakeVoltage = voltage;
    }

    public void requestHold(double voltage){
        requestIdle = false;
        requestIntake = false;
        requestHold = true;  

        holdVoltage = voltage;
    }
}