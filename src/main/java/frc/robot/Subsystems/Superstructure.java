package frc.robot.Subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDConstants;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;

public class Superstructure extends SubsystemBase {
    //private final Intake intake = new Intake(new IntakeIOTalonFX(canIDConstants.intakeMotor, InvertedValue.CounterClockwise_Positive));
    private final Elevator elevator = new Elevator(new ElevatorIOTalonFX());
    //private final Intake handoff = new Intake(new IntakeIOTalonFX(canIDConstants.handoffMotor, InvertedValue.Clockwise_Positive));
    private final Shooter shooter = new Shooter(new ShooterIOTalonFX());

    


    
}
