// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Commands.JogElevator;
import frc.robot.Commands.runIntake;
import frc.robot.Commands.setElevator;
import frc.robot.Commands.shootVelocity;
import frc.robot.Commands.shootVoltage;
import frc.robot.Commands.zeroVelocity;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final CommandXboxController operator = new CommandXboxController(0);

  private final Elevator s_elevator = new Elevator(new ElevatorIOTalonFX());
  private final Shooter s_shooter = new Shooter(new ShooterIOTalonFX());
  //private final Intake s_intake = new Intake(new IntakeIOTalonFX(15, InvertedValue.CounterClockwise_Positive));
  private final Intake s_handoff = new Intake(new IntakeIOTalonFX(16, InvertedValue.Clockwise_Positive));

  

  public RobotContainer() {
    s_elevator.elevatorConfiguration();
    s_shooter.shooterConfiguration();
    s_shooter.shooterSysIdCmd();
    configureBindings();
    
  }

  private void configureBindings() {
    /* 
    operator.b().onTrue(s_elevator.runSysIdCmd());
    operator.y().onTrue(new JogElevator(s_elevator, 4));
    operator.x().onTrue(new JogElevator(s_elevator, -1.5));
    operator.a().onTrue(new JogElevator(s_elevator, 0));
    */
    /* 
    operator.b().onTrue(new setElevator(s_elevator, 0.2, false));
    operator.a().onTrue(new setElevator(s_elevator, 0, false));
    operator.x().onTrue(new setElevator(s_elevator, 0.45, false));
    */
    
    //operator.y().onTrue(s_shooter.shooterSysIdCmd());

    //operator.leftBumper().whileTrue(new runIntake(s_intake, false));

  
    //operator.leftTrigger().whileTrue(new runIntake(s_handoff, true));

    //operator.rightBumper().onTrue(new shootVoltage(s_shooter, 3));
    //operator.rightTrigger().onTrue(new shootVoltage(s_shooter, 0));
    //operator.rightBumper().onTrue(new shootVelocity(s_shooter, s_handoff, false));
    //operator.rightTrigger().onTrue(new shootVelocity(s_shooter, s_handoff, true));
    //operator.a().onTrue(s_shooter.shootVelocity());
    if (operator.a().getAsBoolean() == true){
      s_shooter.shootVelocity();
    }
    else{
      s_shooter.zeroVelocity();
    }
    
    //operator.a().onTrue(new zeroVelocity(s_shooter));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
