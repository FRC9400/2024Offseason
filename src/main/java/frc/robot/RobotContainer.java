// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Intake s_intake = new Intake(new IntakeIOTalonFX(canIDConstants.intakeMotor));
  private final Elevator s_elevator = new Elevator(new ElevatorIOTalonFX(canIDConstants.leftElevatorMotor, canIDConstants.rightElevatorMotor));
  private final Shooter s_shooter = new Shooter(new ShooterIOTalonFX(canIDConstants.handoverMotor, canIDConstants.leftShooterMotor, canIDConstants.rightShooterMotor));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    controller.leftBumper().whileTrue((new InstantCommand(() -> s_intake.requestIntake(1))))
    .onFalse(new InstantCommand(() -> s_intake.requestIdle()));


    }
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

