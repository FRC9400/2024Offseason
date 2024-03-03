// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final CommandXboxController tuner = new CommandXboxController(0);

  private final Intake s_intake = new Intake(new IntakeIOTalonFX(1, InvertedValue.Clockwise_Positive));
  private final Elevator s_elevator = new Elevator(new ElevatorIOTalonFX());
  private final Shooter s_shooter = new Shooter(new ShooterIOTalonFX());
  private final Swerve s_swerve = new Swerve();

  public RobotContainer() {
    s_elevator.elevatorConfiguration();
    s_shooter.shooterConfiguration();
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {

    tuner.a().onTrue(s_elevator.runSysIdCmd());
    tuner.b().onTrue(s_shooter.shooterSysIdCmd());
    tuner.x().onTrue(s_swerve.steerSysIdCmd());
    tuner.y().onTrue(s_swerve.driveSysIdCmd());

  }

  private void configureDefaultCommands() {
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

