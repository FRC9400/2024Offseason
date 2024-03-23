package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.Constants.canIDConstants;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Handoff.HandoffIO;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff.HandoffStates;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Intake.Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter.ShooterStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class Superstructure extends SubsystemBase {
    private Intake s_intake;
    private Handoff s_handoff;
    private Elevator s_elevator;
    private Shooter s_shooter;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    private double stateStartTime = 0;
    private boolean disableElevator = false;

    LoggedTunableNumber handoffShooterVoltage = new LoggedTunableNumber("Superstructure/handoffShooterVoltage", 3);
    LoggedTunableNumber handoffIntakeVoltage = new LoggedTunableNumber("Superstructure/handoffIntkaeVoltage", 1);
    LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superstructure/intakeVoltage", 4);
    LoggedTunableNumber outakeVoltage = new LoggedTunableNumber("Superstructure/outakeVoltage", -3.5);
    LoggedTunableNumber ampShooterVel = new LoggedTunableNumber("Superstructure/ampShooterVel", 3.2);
    LoggedTunableNumber shootMidVel = new LoggedTunableNumber("Superstructure/shootMIDvel", 20);
    LoggedTunableNumber shootRightVel = new LoggedTunableNumber("Superstructure/shootRIGHTvel", 10);
    LoggedTunableNumber shootLeftVel = new LoggedTunableNumber("Superstructure/shootLEFTvel", 20);
    LoggedTunableNumber midRatio = new LoggedTunableNumber("Superstructure/MIDratio", 0.7);
    LoggedTunableNumber rightRatio = new LoggedTunableNumber("Superstructure/RIGHTratio", 2);
    LoggedTunableNumber leftRatio = new LoggedTunableNumber("Superstructure/LEFTratio", 0.5);
    LoggedTunableNumber climbUpHeight = new LoggedTunableNumber("Superstructure/climbUpHeight", 0.45);
    LoggedTunableNumber climbDownHeight = new LoggedTunableNumber("Superstructure/climbDownHeight", 0);

    public Superstructure(IntakeIO intake, HandoffIO handoff, ElevatorIO elevator, ShooterIO shooter) {
        this.s_intake = new Intake(intake);
        this.s_handoff = new Handoff(handoff);
        this.s_elevator = new Elevator(elevator);
        this.s_shooter = new Shooter(shooter);
    }

    public enum SuperstructureStates {
        IDLE,
        HOMING,
        INTAKE,
        SPIN_UP_AMP,
        SPIN_UP_MID,
        SPIN_UP_LEFT,
        SPIN_UP_RIGHT,
        AMP_SHOOTER,
        SHOOT_RIGHT,
        SHOOT_MID,
        SHOOT_LEFT,
        PASS,
        PREPARE_AMP_ELEVATOR,
        AMP_ELEVATOR,
        EXIT_AMP_ELEVATOR,
        CLIMB_UP,
        CLIMB_DOWN
    }

    @Override
    public void periodic() {
        s_intake.Loop();
        s_elevator.Loop();
        s_handoff.Loop();
        s_shooter.Loop();
        Logger.recordOutput("DisabledElevator", disableElevator);

        Logger.recordOutput("SuperstructureState", systemState);
        switch (systemState) {
            case IDLE:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                break;
            case HOMING:
                s_elevator.setState(ElevatorState.HOMING);

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25
                        && Math.abs(s_elevator.getElevatorVelMPS()) < 0.2) {
                    s_elevator.setState(ElevatorState.ZEROSENSOR);
                    setState(SuperstructureStates.IDLE);
                } else if (s_elevator.getState() == ElevatorState.IDLE) {
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case INTAKE:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(0, 0);
                s_intake.requestIntake(intakeVoltage.get());
                s_handoff.requestHandoff(handoffIntakeVoltage.get());

                if (s_handoff.getStatorCurrent() > 5 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case SPIN_UP_AMP:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(ampShooterVel.get(), 1);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                if (Math.abs(s_shooter.getLeftShooterSpeedMPS() - ampShooterVel.get()) < 0.2) {
                    setState(SuperstructureStates.AMP_SHOOTER);
                }
                break;
            case SPIN_UP_MID:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(shootMidVel.get(), 0.7);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                if (Math.abs(s_shooter.getLeftShooterSpeedMPS() - shootMidVel.get()) < 0.2) {
                    setState(SuperstructureStates.SHOOT_MID);
                }
                break;

            case SPIN_UP_LEFT:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(shootLeftVel.get(), leftRatio.get());
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                if (Math.abs(s_shooter.getLeftShooterSpeedMPS() - shootLeftVel.get()) < 0.2) {
                    setState(SuperstructureStates.SHOOT_LEFT);
                }
                break;

            case SPIN_UP_RIGHT:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(shootRightVel.get(), rightRatio.get());
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                if (Math.abs(s_shooter.getRightShooterSpeedMPS() - shootRightVel.get() * rightRatio.get()) < 0.2) {
                    setState(SuperstructureStates.SHOOT_RIGHT);
                }
                break;

            case AMP_SHOOTER:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(ampShooterVel.get(), 1);
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case SHOOT_RIGHT:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(shootRightVel.get(), rightRatio.get());
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SHOOT_MID:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(20, 0.7);
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SHOOT_LEFT:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(shootLeftVel.get(), leftRatio.get());
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PASS:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(10, 1);
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PREPARE_AMP_ELEVATOR:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0.45, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                if(s_elevator.atElevatorSetpoint(0.45)){
                    setState(SuperstructureStates.AMP_ELEVATOR);
                }
                break;
            case AMP_ELEVATOR:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0.45, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }

                s_shooter.requestVelocity(0, 0);
                if (s_elevator.atElevatorSetpoint(0.44)) {
                    s_intake.requestOutake(outakeVoltage.get());
                }
                s_handoff.setState(HandoffStates.IDLE);

                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.75) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case CLIMB_UP:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(0.45, false);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);
                break;
            case CLIMB_DOWN:
                if (!disableElevator) {
                    s_elevator.requestElevatorHeight(climbDownHeight.get(), true);
                } else {
                    s_elevator.setState(ElevatorState.IDLE);
                }
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);
                break;
            default:
                break;
        }
    }

    public void setState(SuperstructureStates nextState) {
        this.systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public void disablingElevator() {
        disableElevator = true;
    }

    public double getIntakeCurrent(){
        return s_intake.getStatorCurrent();
    }

    public SuperstructureStates getState() {
        return systemState;
    }
}
