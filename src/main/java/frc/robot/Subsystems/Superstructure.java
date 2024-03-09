package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDConstants;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff.HandoffStates;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Intake.Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;

public class Superstructure extends SubsystemBase {
  private Intake s_intake;
  private Handoff s_handoff;
  private Elevator s_elevator;
  private Shooter s_shooter;
  private SuperstructureStates systemState = SuperstructureStates.IDLE;

  private boolean idle = true;
  private boolean homing = false;
  private boolean intake = false;
  private boolean amp_shooter = false;
  private boolean shoot_right = false;
  private boolean shoot_left = false;
  private boolean shoot_mid = false;
  private boolean prepare_amp_elevator = false;
  private boolean amp_elevator = false;
  private boolean exit_amp_elevator = false;
  private boolean climb_up = false;
  private boolean climb_down = false;

  public Superstructure(Intake s_intake, Handoff s_handoff, Elevator s_elevator, Shooter s_shooter) {
    this.s_intake = s_intake;
    this.s_handoff = s_handoff;
    this.s_shooter = s_shooter;
    this.s_elevator = s_elevator;
  }

  public enum SuperstructureStates{
    IDLE,
    HOMING,
    INTAKE,
    AMP_SHOOTER,
    SHOOT_RIGHT,
    SHOOT_MID,
    SHOOT_LEFT,
    PREPARE_AMP_ELEVATOR,
    AMP_ELEVATOR,
    EXIT_AMP_ELEVATOR,
    CLIMB_UP,
    CLIMB_DOWN
  }
  
  @Override
  public void periodic(){
    Logger.recordOutput("SuperstructureState", systemState);
    switch(systemState){
            case IDLE: 
                s_elevator.requestElevatorHeight(0);
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);
                
                if(homing){
                    setState(SuperstructureStates.HOMING);
                }
                else if(intake){
                    setState(SuperstructureStates.INTAKE);
                }
                else if(amp_shooter){
                    setState(SuperstructureStates.AMP_SHOOTER);
                }
                else if(shoot_right){
                    setState(SuperstructureStates.SHOOT_RIGHT);
                }
                else if(shoot_left){
                    setState(SuperstructureStates.SHOOT_LEFT);
                }
                else if(shoot_mid){
                    setState(SuperstructureStates.SHOOT_MID);
                }
                else if (prepare_amp_elevator){
                    setState(SuperstructureStates.PREPARE_AMP_ELEVATOR);
                }
                else if(climb_up){
                    setState(SuperstructureStates.CLIMB_UP);
                }


                break;
            case HOMING: 
                s_elevator.setState(ElevatorState.HOMING);
                
                if(s_elevator.getState() == ElevatorState.IDLE){
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case INTAKE:
                s_elevator.requestElevatorHeight(0);
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.INTAKE);
                s_handoff.setState(HandoffStates.IDLE);

                
                break;
            case AMP_SHOOTER:    
                break;
            case SHOOT_RIGHT:        
                break;
            case SHOOT_MID:
                break;
            case SHOOT_LEFT:
                break; 
            case PREPARE_AMP_ELEVATOR:
                break;
            case AMP_ELEVATOR:
                break;
            case CLIMB_UP:
                break;
            case CLIMB_DOWN:
                break;
            default:
                break;
        }
  }
  
  public void setState(SuperstructureStates nextState){
    this.systemState = nextState;
  }
  
    
}
