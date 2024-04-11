package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class TWO_PIECE_SOURCE extends SequentialCommandGroup{
    private final PathPlannerPath forward = PathPlannerPath.fromChoreoTrajectory("TwoNoteSourceForward");
    private final PathPlannerPath backward = PathPlannerPath.fromChoreoTrajectory("TwoNoteSourceBackward");
    public TWO_PIECE_SOURCE(Swerve swerve, Superstructure superstructure){
        
        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.setGyroStartingPosition(60)),
        //new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d( 0.7651792764663696, 4.407987117767334 ), new Rotation2d(-1.047047697709670)))),
        new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d( 15.7163724899292, 4.378147125244141 ), new Rotation2d(1.047)))),
        AutoBuilder.followPath(forward),
        //.raceWith(new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A))),
        new WaitCommand(2.5),
        AutoBuilder.followPath(backward)
        );
        
    }
}

