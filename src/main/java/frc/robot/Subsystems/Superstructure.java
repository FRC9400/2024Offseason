package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.shooterConstants;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.Shooter.ShooterArm;
import frc.robot.Subsystems.Shooter.ShooterArmIO;


public class Superstructure extends SubsystemBase{
    private Indexer s_indexer;
    private OTB_Intake s_intake;
    private ShooterArm s_shooter;
    private Amp s_amp;
    private double stateStartTime = 0;

    private SuperstructureStates systemState = SuperstructureStates.IDLE;
    
    LoggedTunableNumber handoffShooterVoltage = new LoggedTunableNumber("Superstructure/handoffVoltage", 3);
    LoggedTunableNumber handoffIntakeVoltage = new LoggedTunableNumber("Superstructure/handoffIntkaeVoltage", 1);
    LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superstructure/intakeVoltage", 4);
    LoggedTunableNumber outakeVoltage = new LoggedTunableNumber("Superstructure/outakeVoltage", -3.5);
    LoggedTunableNumber ampShooterVel = new LoggedTunableNumber("Superstructure/ampShooterVel", 3.2);
    LoggedTunableNumber shootMidVel = new LoggedTunableNumber("Superstructure/shootMIDvel", 20);
    LoggedTunableNumber shootRightVel = new LoggedTunableNumber("Superstructure/shootRIGHTvel", 15);
    LoggedTunableNumber shootLeftVel = new LoggedTunableNumber("Superstructure/shootLEFTvel", 20);

    public Superstructure(IndexerIO indexer, OTB_IntakeIO intake, ShooterArmIO shooter){
        this.s_indexer = new Indexer(indexer);
        this.s_intake = new OTB_Intake(intake);
        this.s_shooter = new ShooterArm(shooter);
    }

    public enum SuperstructureStates{
        IDLE,
        INTAKE,
        HANDOFF,
        AMP_A,
        AMP_B,
        SHOOT,
        PASS,
        PREPARE_SHOOT,
        TEST_SHOT_VOLTAGE_PREPARE,
        TEST_SHOOT_VOLTAGE,
        TEST_IDLE
    }

    @Override
    public void periodic(){
        s_indexer.Loop();
        s_intake.Loop();
        s_shooter.Loop();
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestZero();
                break;
            case INTAKE:
                s_indexer.requestHandoff(2);
                s_amp.requestRun(2);
                s_intake.requestIntake();
                s_shooter.requestZero();
                if (s_indexer.getIndexerCurrent() > 40 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.HANDOFF);
                }
                break;
            case HANDOFF:
                s_indexer.requestHandoff(2);//placeholder value(s)
                s_amp.requestRun(2);
                s_intake.requestIdle();
                s_shooter.requestIdle();
                if (s_amp.getAmpCurrent() > 40 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case AMP_A:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                if(Math.abs(s_shooter.getArmDegrees() - 130) < 0.1){
                    setState(SuperstructureStates.AMP_B);
                }
                break;
            case AMP_B:
                s_indexer.requestIdle();//placeholder value(s)
                s_amp.requestRun(-3);
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.75) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SHOOT:
                s_indexer.requestIdle();
                s_amp.requestRun(2);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestShoot(20, 0.5,30);//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PASS:
                //s_indexer.requestHandoff(0, 0);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestPass(0,0, 0);//placeholder value(s)
                break;
            case PREPARE_SHOOT:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(20, 0.5, 30);//placeholder value(s)
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            }
        }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

    public void requestAmp(){
        setState(SuperstructureStates.AMP_A);
    }

    public void requestIntake(){
        setState(SuperstructureStates.INTAKE);
    }

    public void requestPreShoot(){
        setState(SuperstructureStates.PREPARE_SHOOT);
    }

    public void requestPass(){
        setState(SuperstructureStates.PASS);
    }
    

    public void setState(SuperstructureStates next){
        systemState=next;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}