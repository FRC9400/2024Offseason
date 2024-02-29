package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final SysIdRoutine elevatorRoutine;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorState state = ElevatorState.IDLE;
    private double elevatorSetpoints[] = {0, 0}; //stow setpoint, setpoint
    private double startTime = 0;

    public enum ElevatorState{
        IDLE,
        HOMING, 
        JOG,
        SETPOINT,
        STOW,
        CLIMB
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(inputs);
        elevatorIO.updateTunableNumbers();
        Logger.processInputs("Elevator", inputs);

        switch(state){
            case IDLE:
                elevatorIO.setOutput(0);
                break;
            case HOMING:
                elevatorIO.setOutput(-4);
                if(RobotController.getFPGATime()/1.0E6 - 0 > 0.5 && Math.abs(inputs.elevatorVelMPS) < 0.1){
                    elevatorIO.zeroSensor();
                    setState(ElevatorState.IDLE);
                }
                break;
            case JOG:
                break;
            case SETPOINT:
                break;
            case STOW:
                break;
            case CLIMB:
                break;
        }
    }

    public void setState(ElevatorState nextState){

    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> elevatorIO.setOutput(volts.in(Volts)), null,
                        this));
    }

    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                elevatorRoutine
                        .quasistatic(Direction.kForward)
                        .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.02),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),
                elevatorRoutine
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.elevatorHeightMeters < 0.02),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),

                elevatorRoutine
                        .dynamic(Direction.kForward)
                        .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.02),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),

                elevatorRoutine
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.elevatorHeightMeters < 0.02),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    }

    public void setOutput(double output) {
        elevatorIO.setOutput(output);
    }

    public void driveElevator(double x, boolean climb) {
        elevatorIO.driveElevator(x, climb);
    }

    public void elevatorConfiguration() {
        elevatorIO.elevatorConfiguration();
    }



}
