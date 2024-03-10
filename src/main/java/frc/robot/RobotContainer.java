// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.InstanceAlreadyExistsException;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Handoff.HandoffIO;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  private final IntakeIO s_intake = new IntakeIOTalonFX();
  private final HandoffIO s_handoff = new HandoffIOTalonFX();
  private final ElevatorIO s_elevator = new ElevatorIOTalonFX();
  private final ShooterIO s_shooter = new ShooterIOTalonFX();
  private final Superstructure superstructure = new Superstructure(s_intake, s_handoff, s_elevator, s_shooter);
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
    new JoystickButton(operator, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.HOMING)));

    new JoystickButton(operator, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)));

    new JoystickButton(operator, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_RIGHT)));

    new JoystickButton(operator, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_LEFT)));

    new JoystickButton(operator, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)));

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_SHOOTER)));

    new JoystickButton(controller, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_AMP_ELEVATOR)));

    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.CLIMB_UP)));

    new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.CLIMB_DOWN)));
    
  }
  private void configureDefaultCommands() {
   
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

