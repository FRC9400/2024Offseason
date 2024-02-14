// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Elevator.JogElevator;
import frc.robot.Constants.canIDConstants;

public class RobotContainer {
  private final CommandXboxController elevatorTest = new CommandXboxController(0);

  private final Elevator s_elevator = new Elevator(new ElevatorIOTalonFX(canIDConstants.leftElevatorMotor, canIDConstants.rightElevatorMotor));
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    elevatorTest.leftBumper().onTrue(new JogElevator(s_elevator, 2));
    elevatorTest.rightBumper().onTrue(new JogElevator(s_elevator, -1.5));
    elevatorTest.leftTrigger().onTrue(new JogElevator(s_elevator, 0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
