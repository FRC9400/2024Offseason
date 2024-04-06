package frc.robot.autons.modes;

import java.nio.file.Path;

import org.json.simple.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class TWOPieceMid extends SequentialCommandGroup{
    private final PathPlannerPath midForwardPath = PathPlannerPath.fromChoreoTrajectory("forward");
    private final PathPlannerPath backPath = PathPlannerPath.fromChoreoTrajectory("backtestpath");
    private final PathPlannerPath turn = PathPlannerPath.fromChoreoTrajectory("turn");
    private final PathPlannerPath centerline = PathPlannerPath.fromChoreoTrajectory("centerline_2");
    private final PathPlannerPath note1 = PathPlannerPath.fromChoreoTrajectory("note1");
    private final PathPlannerPath back1 = PathPlannerPath.fromChoreoTrajectory("back1");
    public TWOPieceMid(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(1.41186296939849839, 5.5722), new Rotation2d(0)))),
            AutoBuilder.followPath(note1),
            new WaitCommand(0.5),
            AutoBuilder.followPath(back1)
            
            //AutoBuilder.followPath(midForwardPath),
            //new WaitCommand(1),
            //AutoBuilder.followPath(backPath)
    
            );
        
    }
}

