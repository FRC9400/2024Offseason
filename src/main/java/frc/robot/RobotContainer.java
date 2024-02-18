// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Commands.JogElevator;
import frc.robot.Commands.shootVoltage;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final CommandXboxController operator = new CommandXboxController(0);

  private final Elevator s_elevator = new Elevator(new ElevatorIOTalonFX());
  private final Shooter s_shooter = new Shooter(new ShooterIOTalonFX());
  

  public RobotContainer() {
    s_elevator.elevatorConfiguration();
    s_shooter.shooterConfiguration();
    configureBindings();
    
  }

  private void configureBindings() {
    
    operator.b().onTrue(new JogElevator(s_elevator, 2));
    operator.x().onTrue(new JogElevator(s_elevator, -1.5));
    operator.a().onTrue(new JogElevator(s_elevator, 0));

    operator.rightBumper().onTrue(new shootVoltage(s_shooter, 3));
    operator.rightTrigger().onTrue(new shootVoltage(s_shooter, 0));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
