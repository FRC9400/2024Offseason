// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.autons.modes.Autos;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    public static final CommandXboxController controller = new CommandXboxController(0);
    private final ShooterArmIO shooter = new ShooterArmIOTalonFX();
    private final OTB_IntakeIO otbIntake = new OTB_IntakeIOTalonFX();
    private final IndexerIO indexer = new IndexerIOTalonFX();
    private final AmpIO amp = new AmpIOTalonFX();
    private final Superstructure superstructure = new Superstructure(indexer, otbIntake, shooter, amp);
    private final Swerve swerve = new Swerve();
    private AutonomousSelector selector;
  public RobotContainer() {
    configureAutonomousSelector();
  
    swerve.zeroWheels();
    swerve.zeroGyro();
    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -controller.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -controller.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -controller.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );
    configureBindings();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption(
      "Preload Mid", Autos.preloadMid(swerve, superstructure)
      );
    autoChooser.addOption(
      "Preload Amp", Autos.preloadAmp(swerve, superstructure)
      );
    autoChooser.addOption(
      "Preload Source", Autos.preloadSource(swerve, superstructure)
      );
    autoChooser.addOption(
      "Four Note Mid", Autos.fourNoteMid(swerve, superstructure)
      );
    autoChooser.addOption(
      "Four Note Amp", Autos.fourNoteAmp(swerve, superstructure)
      );
    autoChooser.addOption(
      "Four Note Source", Autos.fourNoteSource(swerve, superstructure)
    );
    autoChooser.addOption(
      "Test", Autos.TestAuto(swerve, superstructure, "mid", Choreo.getTrajectory("test"))
      );
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    controller.leftBumper().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)));
    controller.rightBumper().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_SHOOT)));
    controller.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_A)));
    controller.a().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE)));


  }

  public modes getAutoCommand() {
    return selector.get();
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureAutonomousSelector(){
    selector = new AutonomousSelector(swerve, superstructure);
  }

  public Superstructure getSuperstructure(){
    return superstructure;
  }

  public Swerve getSwerve(){
    return swerve;
  }

}

