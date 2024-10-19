package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.shooterConstants;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Amp.AmpIO;
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
    
    
    LoggedTunableNumber shooterAngle = new LoggedTunableNumber("Superstructure/shootRIGHTvel", 23);
    LoggedTunableNumber shootLeftVel = new LoggedTunableNumber("Superstructure/shootLEFTvel", 70);
    LoggedTunableNumber passAngle = new LoggedTunableNumber("Superstructure/passAngle", 23);
    LoggedTunableNumber passVel = new LoggedTunableNumber("Superstructure/passVel", 70);


    public Superstructure(IndexerIO indexer, OTB_IntakeIO intake, ShooterArmIO shooter, AmpIO amp){
        this.s_indexer = new Indexer(indexer);
        this.s_intake = new OTB_Intake(intake);
        this.s_shooter = new ShooterArm(shooter);
        this.s_amp = new Amp(amp);
    }

    public enum SuperstructureStates{
        IDLE,
        INTAKE,
        NOTE,
        OUTTAKE,
        HANDOFF_A,
        HANDOFF_B,
        AMP_A,
        AMP_B,
        SHOOT,
        PASS,
        PRE_PASS,
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
        s_amp.Loop();
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
                s_shooter.requestHandoff(6, 1);
                if (s_amp.getAmpCurrent() > 15 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.HANDOFF_A);
                }
                break;
            case NOTE:
                RobotContainer.controller.setRumble(RumbleType.kLeftRumble, 1);
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                    RobotContainer.controller.setRumble(RumbleType.kLeftRumble, 0);
                }
                break;
            case OUTTAKE:
                s_indexer.requestHandoff(-2);
                s_amp.requestRun(-2);
                s_intake.requestOuttake();
                break;
            case HANDOFF_A:
                s_indexer.requestHandoff(2);//placeholder value(s)
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestHandoff(6, 1);
                if (s_shooter.getCurrent() > 40 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.HANDOFF_B);
                }
                break;
            case HANDOFF_B:
                s_indexer.requestIdle();//placeholder value(s)
                s_amp.requestRun(-2);
                s_intake.requestSetpoint();
                s_shooter.requestHandoff(-6, 1);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.3) {
                    setState(SuperstructureStates.NOTE);
                }
                break;
            case AMP_A:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                
                break;
            case AMP_B:
                s_indexer.requestIdle();
                s_amp.requestRun(-3);
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.75) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PREPARE_SHOOT:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(shootLeftVel.get(), 0.5,shooterAngle.get());//placeholder value(s)
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            case SHOOT:
                s_indexer.requestIdle();
                s_amp.requestRun(2);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestShoot(shootLeftVel.get(), 0.5,shooterAngle.get());//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PRE_PASS:
                //s_indexer.requestHandoff(0, 0);//placeholder value(s)
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(passVel.get(), 0.5,passAngle.get());//placeholder value(s)
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            case PASS:
                s_indexer.requestIdle();
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestShoot(passVel.get(), 0.5,passAngle.get());//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
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