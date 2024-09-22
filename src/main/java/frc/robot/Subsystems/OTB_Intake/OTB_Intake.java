
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

    public OTB_Intake(OTB_IntakeIO otbIntakeIO) {
        this.otbIntakeIO = otbIntakeIO;
        pivotSysID  = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(2), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> otbIntakeIO.requestPivotVoltage(volts.in(Volts)), null,
                    this));
      }
      
    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < -30),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(() -> inputs.pivotPosDeg > 0),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < -30),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(() -> inputs.pivotPosDeg > 0),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    } 
    
    @Override
    public void periodic() {
        otbIntakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }


}
