// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Amp.AmpIO;
import frc.robot.Subsystems.Amp.AmpIOTalonFX;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.Indexer.IndexerIOTalonFX;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.ShooterArm;
import frc.robot.Subsystems.Shooter.ShooterArmIO;
import frc.robot.Subsystems.Shooter.ShooterArmIOTalonFX;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.AmpDriveAssistCommand;
import frc.robot.Subsystems.Swerve.PassAssistCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
  public static final XboxController controller = new XboxController(0);
  public static final CommandXboxController driver = new CommandXboxController(1);
    private final ShooterArmIO shooter = new ShooterArmIOTalonFX();
    private final OTB_IntakeIO otbIntake = new OTB_IntakeIOTalonFX();
    private final IndexerIO indexer = new IndexerIOTalonFX();
    private final AmpIO amp = new AmpIOTalonFX();
    private final Superstructure superstructure = new Superstructure(indexer, otbIntake, shooter,amp);
    private final Swerve swerve = new Swerve();

    public final EventLoop m_loop = new EventLoop();//¯\_(ツ)_/¯
  public RobotContainer() {
  
    swerve.zeroWheels();
    swerve.zeroGyro();
    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -driver.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );
    configureBindings();


  }

  private void configureBindings() {
    /*controller.leftBumper(onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE))));
    controller.rightBumper(onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_SHOOT))));
    controller.leftTrigger(onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_A))));
    controller.a(onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE))));
    controller.b(onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.OUTTAKE))));*/
    driver.x().onTrue(new InstantCommand(() -> swerve.zeroWheels()));
    driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    /* 
    BooleanEvent leftBumper = new BooleanEvent(m_loop, () -> controller.getLeftBumper());
    leftBumper.ifHigh(() -> superstructure.setState(SuperstructureStates.INTAKE));

    BooleanEvent bPressed = new BooleanEvent(m_loop, () -> controller.getBButton());
    bPressed.ifHigh(() -> superstructure.setState(SuperstructureStates.OUTTAKE));
    */
    //driver.rightBumper().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_SHOOT)));
    driver.leftTrigger().whileTrue(new AmpDriveAssistCommand(swerve, superstructure));
    driver.leftBumper().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_B)));
    driver.rightTrigger().whileTrue(new PassAssistCommand(swerve, superstructure));
    driver.a().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE)));
    driver.rightBumper().onTrue(new InstantCommand(()-> superstructure.setState(SuperstructureStates.INTAKE)));
    driver.b().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_SHOOT)));

  }
  public boolean getAutonomousCommand() {
    return false;
    
  }

  public Superstructure getSuperstructure(){
    return superstructure;
  }

  public Swerve getSwerve(){
    return swerve;
  }
}

