package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.Shooter.ShooterStates;

public class Superstructure extends SubsystemBase{
    private OTB_Intake s_intake;
    private Shooter s_shooter;

    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    public Superstructure(OTB_IntakeIO intake, ShooterIO shooter){
        this.s_intake=new OTB_Intake(intake);
        this.s_shooter = new Shooter(shooter);
    }

    public enum SuperstructureStates{
        IDLE,
        INTAKE_MOVE,
        INTAKE_RUN,
        PREPARE_SHOOTER,
        RUN_SHOOTER
    }

    @Override
    public void periodic(){
        s_intake.Loop();
        s_intake.Loop();

        switch(systemState){
            case IDLE:
                s_intake.setState(IntakeStates.IDLE); //setpoint
                s_shooter.setState(ShooterStates.IDLE); 
                break;
            case INTAKE_MOVE:
                s_intake.RequestSetpoint(0);
                s_shooter.setState(ShooterStates.IDLE);
                break;
            case INTAKE_RUN:
                s_intake.RequestIntake(90, 5);
                s_shooter.setState(ShooterStates.IDLE);
                break;
            case PREPARE_SHOOTER:
                s_intake.setState(IntakeStates.IDLE);
                s_shooter.prepShoot(20, 0);//placeholders
                break;
            case RUN_SHOOTER:
                s_intake.shoot(5);
                s_shooter.shoot(20,0);//placeholder values
                break;
        }
    }

    public void setState(SuperstructureStates state){
        this.systemState = state;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}
