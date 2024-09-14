package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
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
    private double stateStartTime = 0;

    private SuperstructureStates systemState = SuperstructureStates.TEST_IDLE;

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
                s_intake.requestSetpoint();
                s_shooter.requestZero();
                break;
            case INTAKE:
                s_indexer.requestHandoff(0, 0);
                s_intake.requestIntake();
                s_shooter.requestZero();
                if(s_indexer.getAmpCurrent()>40 && stateStartTime > 0.3){//placeholder value
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case HANDOFF:
                s_indexer.requestHandoff(0, 0);//placeholder value(s)
                s_intake.requestIdle();
                s_shooter.requestIdle();
                break;
            case AMP_A:
                s_indexer.requestIdle();//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                if(Math.abs(s_shooter.getArmDegrees() - 140) < 0.1){
                    setState(SuperstructureStates.AMP_B);
                }
                break;
            case AMP_B:
                s_indexer.requestAmp(0);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                break;
            case SHOOT:
                s_indexer.requestShoot(0);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestShoot(0, 0,0);//placeholder value(s)
                if(stateStartTime > 2){//placeholder value
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PASS:
                s_indexer.requestHandoff(0, 0);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestPass(0,0, 0);//placeholder value(s)
                break;
            case PREPARE_SHOOT:
                s_indexer.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(0, shooterConstants.shooterGearRatio, 0);//placeholder value(s)
                if(s_shooter.atShooterSetpoint() & s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            case TEST_SHOT_VOLTAGE_PREPARE:
                s_indexer.requestIdle();
                s_intake.requestIdle();
                s_shooter.requestVoltage();
                if( stateStartTime < 5){
                    setState(SuperstructureStates.TEST_SHOOT_VOLTAGE);
                }
                break;
                case TEST_SHOOT_VOLTAGE:
                s_indexer.requestShoot(4);
                s_intake.requestIdle();
                s_shooter.requestVoltage();
                
                break;
                case TEST_IDLE:
                    s_indexer.requestIdle();
                    s_intake.requestIdle();
                    s_shooter.requestIdle();
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