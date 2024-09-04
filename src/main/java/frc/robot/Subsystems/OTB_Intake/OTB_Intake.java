
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


public class OTB_Intake extends SubsystemBase{
    private final OTB_IntakeIO otbIntakeIO;
    private OTB_IntakeIOInputsAutoLogged inputs = new OTB_IntakeIOInputsAutoLogged();
    private OTB_IntakeStates state = OTB_IntakeStates.IDLE;
    private double angleSetpoint = 0;
    private double voltageSetpoint = 0;

    public enum OTB_IntakeStates{
        IDLE,
        INTAKE,
        HOMING,
        SETPOINT,
        ZEROPOSITION
    }

    public OTB_Intake(OTB_IntakeIO otbIntakeIO) {
        this.otbIntakeIO = otbIntakeIO;
      }
      
    @Override
    public void periodic() {
        otbIntakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
        //Logger.recordOutput("OTB_IntakeState", state.toString());
        switch(state){
                case IDLE:
                    otbIntakeIO.requestPivotVoltage(0);
                    otbIntakeIO.requestIntakeVoltage(0);
                    break;
                case HOMING:
                    otbIntakeIO.requestPivotVoltage(-1);
                    otbIntakeIO.requestIntakeVoltage(0);
                    break;
                case INTAKE:
                    otbIntakeIO.requestSetpoint(angleSetpoint);
                    otbIntakeIO.requestIntakeVoltage(voltageSetpoint);
                    break;
                case SETPOINT:
                    otbIntakeIO.requestSetpoint(angleSetpoint);
                    otbIntakeIO.requestIntakeVoltage(0);
                    break;
                case ZEROPOSITION:
                    otbIntakeIO.zeroPosition();
                    otbIntakeIO.requestIntakeVoltage(0);
            }
    }

    public void requestIntake(double angleSetpointDeg, double voltage){
        this.angleSetpoint = angleSetpointDeg;
        this.voltageSetpoint = voltage;
        setState(OTB_IntakeStates.INTAKE);
    }

    public void requestSetpoint(double angleSetpointDeg){
        this.angleSetpoint = angleSetpointDeg;
        this.voltageSetpoint = 0;
        setState(OTB_IntakeStates.SETPOINT);
    }

    public void requestIdle(){
        setState(OTB_IntakeStates.IDLE);
    }

    public void setState(OTB_IntakeStates nextState){
        this.state = nextState;
    }

    public OTB_IntakeStates getState(){
        return this.state;
    }


}
