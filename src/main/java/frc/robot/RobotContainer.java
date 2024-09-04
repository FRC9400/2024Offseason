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
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIOTalonFX;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
  public static final CommandXboxController controller = new CommandXboxController(0);
    private final Shooter shooter = new Shooter(new ShooterIOTalonFX());
    private final OTB_Intake otbIntake = new OTB_Intake(new OTB_IntakeIOTalonFX());
    private final Indexer indexer = new Indexer(new IndexerIOTalonFX());
    private final Swerve swerve = new Swerve();
  public RobotContainer() {
  
    swerve.zeroWheels();
    swerve.zeroGyro();
    
    configureBindings();

  }

  private void configureBindings() {    

  }

  public boolean getAutonomousCommand() {
    return false;
    
  }

}

