package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Intake.Intake;

public class scoreAmpElevator extends Command {
    Elevator Elevator;
    Intake Intake;
    public scoreAmpElevator(Elevator Elevator, Intake Intake) {
    this.Elevator = Elevator;
    this.Intake = Intake;

    addRequirements(Elevator, Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.driveElevator(0.45);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
    
}
