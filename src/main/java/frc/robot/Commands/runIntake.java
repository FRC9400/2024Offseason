// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class runIntake extends Command {
  private final Intake Intake;
  private double voltage;
  private boolean outake;
  /** Creates a new runIntake. */
  public runIntake(Intake Intake, double voltage, boolean outake) {
    this.Intake = Intake;
    this.outake = outake;
    if(outake){
      this.voltage = voltage * -1;
    }
    
    addRequirements(Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.spinIntake(voltage);
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
