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
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.runIntake;
import frc.robot.Commands.setElevator;
import frc.robot.Commands.shootVelocity;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Intake s_intake = new Intake(new IntakeIOTalonFX());
  private final Handoff s_handoff = new Handoff(new HandoffIOTalonFX());
  private final Elevator s_elevator = new Elevator(new ElevatorIOTalonFX());
  private final Shooter s_shooter = new Shooter(new ShooterIOTalonFX());
  private final Swerve s_swerve = new Swerve();
  public RobotContainer() {
    s_elevator.elevatorConfiguration();
    s_shooter.shooterConfiguration();
    s_swerve.zeroWheels();
    s_swerve.zeroGyro();
    s_swerve.setDefaultCommand(
            new TeleopSwerve(
                s_swerve, 
                () -> -controller.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -controller.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -controller.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );
    
    configureBindings();
    configureDefaultCommands();

  }

  private void configureBindings() {
    controller.rightTrigger().onTrue(new shootVelocity(s_shooter, s_handoff, s_intake, false, 3, 1)); // shoot amp
    controller.leftTrigger().onTrue(new shootVelocity(s_shooter, s_handoff, s_intake, true, 0, 0)); // zero

    controller.x().onTrue(new shootVelocity(s_shooter, s_handoff, s_intake, false, 20, 0.5));
    controller.y().onTrue(new shootVelocity(s_shooter, s_handoff, s_intake, false, 20, 0.7));
    controller.b().onTrue(new shootVelocity(s_shooter, s_handoff, s_intake, false, 10, 2));

   controller.rightBumper().whileTrue(new runIntake(s_intake, false));
   controller.leftBumper().whileTrue(new runIntake(s_intake, true));

   //controller.x().onTrue(new setElevator(s_elevator, 0.45, false));
   //controller.y().onTrue(new setElevator(s_elevator, 0, false));
  }
  private void configureDefaultCommands() {
    
   
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

