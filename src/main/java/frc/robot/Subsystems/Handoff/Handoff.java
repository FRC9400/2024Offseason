
package frc.robot.Subsystems.Handoff;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Handoff extends SubsystemBase{
    private final HandoffIO handoffIO;
    private HandoffIOInputsAutoLogged inputs = new HandoffIOInputsAutoLogged();
    private double handoffVoltage = 0.0;
    

    public Handoff(HandoffIO handoffIO) {
        this.handoffIO = handoffIO;
      }
    
    @Override
    public void periodic(){
        handoffIO.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
        Logger.recordOutput("HandoffVoltageSetpoint", handoffVoltage);
    
    }

    
    public void spinHandoff(double volts){
        handoffIO.setHandoffVoltage(volts);
    }

    
}