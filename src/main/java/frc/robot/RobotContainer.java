// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import javax.management.InstanceAlreadyExistsException;
import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.autoConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.Commands.autons.intakeNote;
import frc.robot.Commands.autons.preloadLeft;
import frc.robot.Commands.autons.preloadMid;
import frc.robot.Commands.autons.preloadRight;
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

  // Field2d m_field = new Field2d();

  // ChoreoTrajectory midtraj;
  // ChoreoTrajectory midReturnTraj;

  public RobotContainer() {
    // midtraj = Choreo.getTrajectory("mid");
    // midReturnTraj = Choreo.getTrajectory("mid.1");

    // m_field.getObject("traj").setPoses(
    //   midtraj.getInitialPose(), midtraj.getFinalPose()
    // );
    // m_field.getObject("trajPoses").setPoses(
    //   midtraj.getPoses()
    // );

    //SmartDashboard.putData(m_field);
    //s_elevator.elevatorConfiguration();
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

    new JoystickButton(controller, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)));

    new JoystickButton(operator, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_RIGHT)));

    new JoystickButton(operator, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_LEFT)));

    new JoystickButton(operator, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)));

    new JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_SHOOTER)));

    //new JoystickButton(controller, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_AMP_ELEVATOR)));

    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.CLIMB_UP)));

    new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.CLIMB_DOWN)));

    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE)));
    
  }
  private void configureDefaultCommands() {
   
  } 

  public Command getAutonomousCommand() {
    AutoBuilder.configureHolonomic(
            s_swerve::getPoseRaw, // Robot pose supplier
            s_swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            s_swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            s_swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                    5, // Max module speed, in m/s
                    0.59055/2, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            s_swerve // Reference to this subsystem to set requirements
    );
    // Load a Choreo trajectory as a PathPlannerPath
    PathPlannerPath midPath = PathPlannerPath.fromChoreoTrajectory("mid");
    PathPlannerPath midReturnPath = PathPlannerPath.fromChoreoTrajectory("mid.1");
    Command mid = AutoBuilder.followPath(midPath);
    Command midReturn = AutoBuilder.followPath(midReturnPath);
    Command shootMid = new preloadMid(s_swerve, superstructure);
    Command intakeNote = new intakeNote(s_swerve, superstructure);
    return Commands.sequence(
        Commands.runOnce(() -> s_swerve.resetOdometry(midPath.getPreviewStartingHolonomicPose())),
        shootMid,
        mid,
        intakeNote,
        midReturn,
        shootMid,
        s_swerve.run(() -> s_swerve.requestVoltage(0, 0, 0, true))
    );
    /*var thetaController = new PIDController(autoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    s_swerve.resetOdometry(midtraj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        midtraj, // Choreo trajectory from above
        s_swerve::getPoseRaw, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(autoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(autoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> s_swerve.requestVoltage( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        s_swerve // The subsystem(s) to require, typically your drive subsystem only
    );
    */
  }
  public void periodic(){
    //m_field.setRobotPose(s_swerve.getPoseRaw());
  }
}

