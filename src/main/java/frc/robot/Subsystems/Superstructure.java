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
        INTAKE_DOWN,
        INTAKE_RUN,
        INTAKE_UP,
        SETPOINT_SHOOTER,
        RUN_SHOOTER
    }

    @Override
    public void periodic(){
        s_intake.Loop();
        s_intake.Loop();

        switch(systemState){
            case IDLE:
                s_intake.setState(IntakeStates.IDLE);
                s_shooter.setState(ShooterStates.IDLE);
                break;
            case INTAKE_DOWN:
                s_intake.RequestSetpoint(90);
                s_shooter.setState(ShooterStates.IDLE);
                break;
            case INTAKE_RUN:
                s_intake.RequestIntake(90, 5);
                s_shooter.setState(ShooterStates.IDLE);
                break;
            case INTAKE_UP:
                s_intake.RequestSetpoint(0);
                s_shooter.setState(ShooterStates.IDLE);
                break;
            case SETPOINT_SHOOTER:
                s_intake.setState(IntakeStates.IDLE);
                s_shooter.requestSetpoint(0);
                break;
            case RUN_SHOOTER:
                s_intake.setState(IntakeStates.IDLE);
                s_shooter.requestVelocity(20);
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
