
package frc.robot.Subsystems.Handoff;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;


public class Handoff extends SubsystemBase{
    private final HandoffIO handoffIO;
    private HandoffIOInputsAutoLogged inputs = new HandoffIOInputsAutoLogged();
    private double handoffVoltage = 0.0;
    private HandoffStates state = HandoffStates.IDLE;

    
    public enum HandoffStates{
        IDLE,
        HANDOFF
    }
    

    public Handoff(HandoffIO handoffIO) {
        this.handoffIO = handoffIO;
      }
    
    @Override
    public void periodic(){
        handoffIO.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
        Logger.recordOutput("HandoffState", state);
        switch(state){
            case IDLE:
                handoffIO.setOutput(0);
                break;
            case HANDOFF:
                handoffIO.setOutput(handoffVoltage);
                break;
            default:
                break;
        }
    
    }
    public void requestHandoff(double voltage){
        handoffVoltage = voltage;
        setState(HandoffStates.HANDOFF);
    }

    public void setState(HandoffStates nextState){
        this.state = nextState;
    }

    
}