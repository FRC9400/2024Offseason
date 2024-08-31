
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
    private final SysIdRoutine pivotSysID;
    private double angleSetpoint = 0;
    private double voltsSetpoint = 0;


    private IntakeStates state = IntakeStates.IDLE;

    public enum IntakeStates{
        IDLE,
        SETPOINT,
        INTAKE
    }

    public OTB_Intake(OTB_IntakeIO otbIntakeIO) {
        this.otbIntakeIO = otbIntakeIO;
        pivotSysID  = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> otbIntakeIO.requestPivotVoltage(volts.in(Volts)), null,
                    this));
      }
      
    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 110),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 110),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    } 
    
    public void Loop(){
        otbIntakeIO.updateInputs(inputs);

        switch(state){
            case IDLE:
                otbIntakeIO.requestPivotVoltage(0);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
            case SETPOINT:
                otbIntakeIO.requestSetpoint(angleSetpoint);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
            case INTAKE:
                otbIntakeIO.requestSetpoint(angleSetpoint);
                otbIntakeIO.requestIntakeVoltage(voltsSetpoint);
                break;
        }
    }

    public void setState(IntakeStates nextState){
        this.state = nextState;
    }

    public void RequestIntake(double angle, double volts){
        this.angleSetpoint=angle;
        this.voltsSetpoint=volts;
        setState(IntakeStates.INTAKE);
    }

    public void RequestSetpoint(double angle){
        this.angleSetpoint = angle;
        this.voltsSetpoint = 0;
        setState(IntakeStates.SETPOINT);
    }

    public IntakeStates getState(){return this.state;}

    @Override
    public void periodic() {
        otbIntakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }
}
