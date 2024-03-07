// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class shootVelocity extends Command {
  private final Shooter Shooter;
  private final Handoff handoff;
  private final Intake intake;
  boolean zero;
  double velocity;
  double ratio;
  

  /** Creates a new runIntake. */
  public shootVelocity(Shooter Shooter, Handoff handoff, Intake intake, boolean zero, double velocity, double ratio) {
    this.Shooter = Shooter;
    this.handoff = handoff;
    this.intake = intake;
    this.zero = zero;
    this.velocity = velocity;
    this.ratio = ratio;

    addRequirements(Shooter, handoff, intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(zero){
      Shooter.zeroVelocity();
      handoff.spinHandoff(0);
      intake.spinIntake(0);
    }
    else{
    Shooter.shootVelocity(velocity, ratio);
    handoff.spinHandoff(3);
    intake.spinIntake(2);
    }
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


