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
    
    
    LoggedTunableNumber shooterMidAngle = new LoggedTunableNumber("Superstructure/shootMidAngle", 23);
    LoggedTunableNumber shootMidVel = new LoggedTunableNumber("Superstructure/shootMidvel", 70);
    LoggedTunableNumber passAngle = new LoggedTunableNumber("Superstructure/passAngle", 23);
    LoggedTunableNumber passVel = new LoggedTunableNumber("Superstructure/passVel", 70);
    
    LoggedTunableNumber shootRightVel = new LoggedTunableNumber("Superstructure/shootRIGHTvel", 70);
    LoggedTunableNumber shootLeftVel = new LoggedTunableNumber("Superstructure/shootLEFTvel", 70);

    LoggedTunableNumber AutoShootVelocity = new LoggedTunableNumber("Autos/AutoIntakeVelocity", 20);
    LoggedTunableNumber shootRightAngle = new LoggedTunableNumber("Autos/shootRightAngle", 30);
    LoggedTunableNumber shootLeftAngle = new LoggedTunableNumber("Autos/shootLeftAngle", 30);

    LoggedTunableNumber shootSubwooferMidVel = new LoggedTunableNumber("Autos/shootSubwooferMidVel", 50);
    LoggedTunableNumber shootSubwooferRightVel = new LoggedTunableNumber("Autos/shootSubwooferRightVel", 50);
    LoggedTunableNumber shootSubwooferLeftVel = new LoggedTunableNumber("Autos/shootSubwooferLeftVel", 50);

    LoggedTunableNumber shootSubwooferMidAngle = new LoggedTunableNumber("Autos/shootSubwooferMidAngle", 30);
    LoggedTunableNumber shootSubwooferRightAngle = new LoggedTunableNumber("Autos/shootSubwooferRightAngle", 30);
    LoggedTunableNumber shootSubwooferLeftAngle = new LoggedTunableNumber("Autos/shootSubwooferLeftAngle", 30);

    private double[] autoShooterVelocity = {0,0}; //left vel + ratio
    private double autoArmAngleDegrees = 0;

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
        TEST_IDLE,
        AUTO_IDLE,
        AUTO_INTAKE,
        AUTO_HANDOFF_A,
        AUTO_HANDOFF_B,
        PREPARE_SHOOT_AUTO,
        SHOOT_AUTO
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
                s_shooter.requestShoot(shootMidVel.get(), 0.5,shooterMidAngle.get());//placeholder value(s)
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            case SHOOT:
                s_indexer.requestIdle();
                s_amp.requestRun(2);//placeholder value(s)
                s_intake.requestSetpoint();
                s_shooter.requestShoot(shootMidVel.get(), 0.5,shooterMidAngle.get());//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PRE_PASS:
            
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(80, 0.5, 40);//placeholder value(s)
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.PASS);
                }
                break;
            case PASS:
                s_indexer.requestIdle();
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestShoot(80, 0.5,40);//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case AUTO_IDLE:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestAutos(AutoShootVelocity.get(), 0.5);
                break;
            case AUTO_INTAKE:
                s_indexer.requestHandoff(2);
                s_amp.requestRun(2);
                s_intake.requestIntake();
                s_shooter.requestAutos(AutoShootVelocity.get(), 0.5);
                if (s_amp.getAmpCurrent() > 15 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.AUTO_HANDOFF_A);
                }
                break;
            case AUTO_HANDOFF_A:
                s_indexer.requestHandoff(2);//placeholder value(s)
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestAutos(AutoShootVelocity.get(), 0.5);
                if (s_amp.getAmpCurrent() > 25 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.AUTO_HANDOFF_B);
                }
                break;
            
            case AUTO_HANDOFF_B:
                s_indexer.requestIdle();//placeholder value(s)
                s_amp.requestRun(-3);
                s_intake.requestSetpoint();
                s_shooter.requestAutos(AutoShootVelocity.get(), 0.5);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.3) {
                    setState(SuperstructureStates.AUTO_IDLE);
                }
                break;

            case PREPARE_SHOOT_AUTO:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(autoShooterVelocity[0], autoShooterVelocity[1],autoArmAngleDegrees);
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT_AUTO);
                }
                break;

            case SHOOT_AUTO:
                s_indexer.requestIdle();
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestShoot(autoShooterVelocity[0], autoShooterVelocity[1],autoArmAngleDegrees);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.AUTO_IDLE);
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

    public void requestAutoIntake(){
        setState(SuperstructureStates.AUTO_INTAKE);
    }

    public void requestAutoIdle(){
        setState(SuperstructureStates.AUTO_IDLE);
    }

    public void requestAutoShootRight(){
        autoShooterVelocity[0] = shootRightVel.get();
        autoShooterVelocity[1] = 0.5;
        autoArmAngleDegrees = shootRightAngle.get();
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    public void requestAutoShootLeft(){
        autoShooterVelocity[0] = shootLeftVel.get();
        autoShooterVelocity[1] = 0.5;
        autoArmAngleDegrees = shootLeftAngle.get();
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    public void requestAutoShootMid(){
        autoShooterVelocity[0] = 20;
        autoShooterVelocity[1] = 0.5;
        autoArmAngleDegrees = 30;
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    public void requestAutoShootSubwooferM(){
        autoShooterVelocity[0] = shootSubwooferMidVel.get();
        autoShooterVelocity[1] = 0.5;
        autoArmAngleDegrees = shootSubwooferMidAngle.get();
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    public void requestAutoShootSubwooferL(){
        autoShooterVelocity[0] = shootSubwooferLeftVel.get();
        autoShooterVelocity[1] = 0.5;
        autoArmAngleDegrees = shootSubwooferLeftAngle.get();
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    public void requestAutoShootSubwooferR(){
        autoShooterVelocity[0] = shootSubwooferRightVel.get();
        autoShooterVelocity[1] = 0.5;
        autoArmAngleDegrees = shootSubwooferRightAngle.get();
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    
    

    public void setState(SuperstructureStates next){
        systemState=next;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}