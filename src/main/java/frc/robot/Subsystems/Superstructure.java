package frc.robot.Subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDConstants;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;

public class Superstructure extends SubsystemBase {
  private Intake s_intake;
  private Handoff s_handoff;
  private Elevator s_elevator;
  private Shooter s_shooter;

  public Superstructure(Intake s_intake, Handoff s_handoff, Elevator s_elevator, Shooter s_shooter) {
    this.s_intake = s_intake;
    this.s_handoff = s_handoff;
    this.s_shooter = s_shooter;
    this.s_elevator = s_elevator;
  }

 
  

  

    
}
