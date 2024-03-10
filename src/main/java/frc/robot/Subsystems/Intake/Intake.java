
package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;


public class Intake{
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double[] voltage = {0.0, 0.0, 0.0}; //intake, outake, handoff

    private IntakeStates state = IntakeStates.IDLE;

    
    public enum IntakeStates{
        IDLE,
        INTAKE,
        OUTAKE,
        HANDOFF
    }

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
      }
    
    public void Loop(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("IntakeState", state.toString());

        switch(state){
            case IDLE:
                intakeIO.setOutput(0);
                break;
            case INTAKE:
                intakeIO.setOutput(voltage[0]);
                break;
            case OUTAKE:
                intakeIO.setOutput(voltage[1]);
                break;
            case HANDOFF:
                intakeIO.setOutput(voltage[2]);
                break;
            default:
                break;
            }
        }
        public void requestIntake(double voltage){
            this.voltage[0] = voltage;
            setState(IntakeStates.INTAKE);
        }

        public void requestOutake(double voltage){
            this.voltage[1] = voltage;
            setState(IntakeStates.OUTAKE);
        }

        public void requestHandoff(double voltage){
            this.voltage[2] = voltage;
            setState(IntakeStates.HANDOFF);
        }

        public void setState(IntakeStates nextState){
            this.state = nextState;
        }

        public IntakeStates getState(){
            return this.state;
        }

        
    }

    
   


    
