package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class SOURCE3 extends SequentialCommandGroup{

    private final Pose2d startingPose =  new Pose2d(new Translation2d( 0.708431601524353, 4.3697943687438965 ), new Rotation2d(-1.043047697709670)) ;

    private final double startingAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? -60 : 60;
    private final PathPlannerPath forward =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? PathPlannerPath.fromChoreoTrajectory("source3") : PathPlannerPath.fromChoreoTrajectory("source3");
    private final PathPlannerPath backward = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? PathPlannerPath.fromChoreoTrajectory("source3back") :  PathPlannerPath.fromChoreoTrajectory("source3back");
    public SOURCE3(Swerve swerve, Superstructure superstructure){
        
        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.setGyroStartingPosition(startingAngle)),
        new InstantCommand(() -> swerve.resetPose(startingPose)),
        
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
        new WaitCommand(1.25),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(forward),
        new WaitCommand(2.5),
        AutoBuilder.followPath(backward),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_RIGHT))
        );
        
    }
}

