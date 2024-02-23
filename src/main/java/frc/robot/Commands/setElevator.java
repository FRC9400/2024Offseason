// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;

public class setElevator extends Command {
  private final Elevator Elevator;
  private double setpointMeters;
  private boolean climb;
  /** Creates a new JogElevator. */
  public setElevator(Elevator Elevator, double setPointMeters, boolean climb) {
    this.Elevator = Elevator;
    this.setpointMeters = setPointMeters;

    addRequirements(Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.setHeight(setpointMeters);
    Elevator.setElevator(setpointMeters, climb);
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

