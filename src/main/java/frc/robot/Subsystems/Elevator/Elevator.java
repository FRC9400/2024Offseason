package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ElevatorStates systemState = ElevatorStates.IDLE;
    private ElevatorStates nextSystemState = systemState;

    private boolean requestIdle = true;
    private boolean requestHoming = false;
    private boolean requestJog = false;
    private boolean requestFollowingSetpoint = false;
    private boolean requestClimb = false;

    private double elevatorSetpoint = 0.0;
    private double elevatorVoltage = 0.0;

    private double startTime = 0;

    public enum ElevatorStates {
        IDLE,
        HOMING,
        JOG,
        FOLLOWING_SETPOINT,
        CLIMB
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(inputs);
        elevatorIO.updateTunableNumbers();
        Logger.processInputs("Elevator", inputs);

        if (systemState == ElevatorStates.IDLE) {
            elevatorIO.setVoltage(0);

            if (requestHoming) {

            } else if (requestJog) {

            } else if (requestFollowingSetpoint) {

            } else if (requestClimb) {

            }

        } else if (systemState == ElevatorStates.HOMING) {
            elevatorIO.setVoltage(-1);
            startTime = Timer.getFPGATimestamp();
            if (((Timer.getFPGATimestamp() - startTime) < 0.5) && inputs.rotorVelocityRPS[0] < 0.1) {
                elevatorIO.setVoltage(0);
                elevatorIO.resetHeight();
            }
        }

        else if (systemState == ElevatorStates.JOG) {
            elevatorIO.setVoltage(elevatorVoltage);

            if (requestIdle) {

            } else if (requestHoming) {

            } else if (requestJog) {

            } else if (requestFollowingSetpoint) {

            }

        } else if (systemState == ElevatorStates.FOLLOWING_SETPOINT) {
            elevatorIO.setHeight(elevatorSetpoint);

            if (requestIdle) {

            } else if (requestHoming) {

            } else if (requestJog) {

            } else if (requestFollowingSetpoint) {

            } else if (requestClimb) {

            }

        } else if (systemState == ElevatorStates.CLIMB) {
            elevatorIO.setHeight(elevatorSetpoint);

            if (requestIdle) {

            }
        }

        if (systemState != nextSystemState) {
            systemState = nextSystemState;
        }
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void elevatorConfiguration() {
        elevatorIO.elevatorConfiguration();
    }

}
