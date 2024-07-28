// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.HandoffIO;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
  public static final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private AutonomousSelector selector;
  private final IntakeIO s_intake = new IntakeIOTalonFX();
  private final HandoffIO s_handoff = new HandoffIOTalonFX();
  private final ElevatorIO s_elevator = new ElevatorIOTalonFX();
  private final ShooterIO s_shooter = new ShooterIOTalonFX();
  private final OTB_IntakeIO otbIntake = new OTB_IntakeIOTalonFX();
  private final LEDs led = new LEDs();
  private final Swerve s_swerve = new Swerve();
  public RobotContainer() {
    configureAutonomousSelector();
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
    

  }
  private void configureDefaultCommands() {
   
  } 

  public modes getAutonomousCommand() {
    return selector.get();
    
  }

  public void configureAutonomousSelector(){
    
  }

}

