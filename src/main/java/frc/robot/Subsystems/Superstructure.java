package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.Shooter.ShooterStates;

public class Superstructure extends SubsystemBase{
    private Indexer indexer;
    private OTB_Intake intake;
    private Shooter shooter;

    private SuperstructureStates state = SuperstructureStates.IDLE;

    public Superstructure(IndexerIO indexer, OTB_IntakeIO intake, ShooterIO shooter){
        this.indexer=new Indexer(indexer);
        this.intake=new OTB_Intake(intake);
        this.shooter = new Shooter(shooter);
    }

    public enum SuperstructureStates{
        IDLE,
        INTAKE,
        PREPARE_SHOOT,
        SHOOT,
        AMP
    }

    @Override
    public void periodic(){
        switch(state){
            case IDLE:
                indexer.requestIdle();
                intake.requestSetpoint(0);//placeholder; put intake up position
                shooter.setState(ShooterStates.ZEROPOSITION);//moves the arm back to starting state i hope?
            case INTAKE:
                indexer.requestIdle();
                intake.requestIntake(0, 0);//placeholders
                shooter.setState(ShooterStates.ZEROPOSITION);//^^^
            case PREPARE_SHOOT:
                indexer.requestIdle();
                intake.requestSetpoint(0);//placeholder
                shooter.setState(ShooterStates.SHOOT);
                //if(){
                  //  setState(SuperstructureStates.SHOOT);
                //}
            case SHOOT:
                indexer.requestHandoff(0, 0);//placeholder
                intake.requestSetpoint(0);//placeholder
                shooter.setState(ShooterStates.SHOOT);
            case AMP:
                indexer.requestOutake(0);//placeholder
                intake.requestSetpoint(0);//placeholder
                shooter.setState(ShooterStates.ZEROPOSITION);//kjsadhgjk
        }
    }

    public void setState(SuperstructureStates nextState){
        this.state=nextState;
    }
}
