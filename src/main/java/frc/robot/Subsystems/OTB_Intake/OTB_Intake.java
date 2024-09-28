
package frc.robot.Subsystems.OTB_Intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class OTB_Intake{
    private final OTB_IntakeIO otbIntakeIO;
    private OTB_IntakeIOInputsAutoLogged inputs = new OTB_IntakeIOInputsAutoLogged();
    private IntakeStates intakeState = IntakeStates.IDLE;

    public enum IntakeStates{
        IDLE,
        INTAKE,
        SETPOINT
    }

    public OTB_Intake(OTB_IntakeIO otbIntakeIO) {
        this.otbIntakeIO = otbIntakeIO;
      }


    public void Loop() {
        otbIntakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
        Logger.recordOutput("OTB_IntakeState", this.intakeState);
        switch(intakeState){
            case IDLE:
                otbIntakeIO.requestPivotVoltage(0);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
            case INTAKE:
                otbIntakeIO.requestIntakeVoltage(3);
                otbIntakeIO.requestSetpoint(39.48);
                break;
            case SETPOINT:
                otbIntakeIO.requestIntakeVoltage(0);
                otbIntakeIO.requestSetpoint(0);
            default:
                break;

        }
    }

    public void requestIdle(){
        setState(IntakeStates.IDLE);
    }
    
    public void requestIntake(){
        setState(IntakeStates.INTAKE);
    }

    public void requestSetpoint(){
        setState(IntakeStates.SETPOINT);
    }

    public void setState(IntakeStates nextState){
        this.intakeState = nextState;
    }
      
    
    
    


}
