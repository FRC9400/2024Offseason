// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.autons.modes.ChoreoTuning;
import frc.robot.autons.modes.PreloadAmp;
import frc.robot.autons.modes.PreloadMid;
import frc.robot.autons.modes.PreloadSource;
import frc.robot.autons.modes.TestAuto;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  SequentialCommandGroup test;
  SequentialCommandGroup preloadAmp;
  SequentialCommandGroup preloadMid;
  SequentialCommandGroup preloadSource;
  SequentialCommandGroup tuning;


  private boolean built = false;

  @Override
  public void robotInit() {
    SignalLogger.setPath("/media/sda1/");
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.*/
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.getAlliance().isPresent() && !built){
      test = new TestAuto(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
      preloadAmp = new PreloadAmp(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
      preloadMid = new PreloadMid(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
      preloadSource = new PreloadSource(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
      tuning = new ChoreoTuning(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
      built = true;
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    if(m_robotContainer.getAutonomousCommand() == modes.TEST){
      m_autonomousCommand = test;
    }
    if(m_robotContainer.getAutonomousCommand() == modes.PRELOAD_AMP){
      m_autonomousCommand = preloadAmp;
    }
    if(m_robotContainer.getAutonomousCommand() == modes.PRELOAD_MID){
      m_autonomousCommand = preloadMid;
    }
    if(m_robotContainer.getAutonomousCommand() == modes.PRELOAD_SOURCE){
      m_autonomousCommand = preloadSource;
    }
    if(m_robotContainer.getAutonomousCommand() == modes.TUNING){
      m_autonomousCommand = tuning;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //test.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
