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

public class TWO_PIECE_SOURCE extends SequentialCommandGroup{

    //private final Pose2d startingPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? new Pose2d(new Translation2d( 0.708431601524353, 4.3697943687438965 ), new Rotation2d(-1.047047697709670)) : new Pose2d(new Translation2d( 15.813826560974121, 4.350637844085693 ), new Rotation2d(Math.PI + 1.047));

    //private final double startingAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? -60 : 60;
    private final PathPlannerPath forward =  PathPlannerPath.fromChoreoTrajectory("sourceC");
    private final PathPlannerPath backward = PathPlannerPath.fromChoreoTrajectory("backSourceC");
    
    public TWO_PIECE_SOURCE(Swerve swerve, Superstructure superstructure){
        
        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.setGyroStartingPosition(60)),
        new InstantCommand(() -> swerve.resetPose( new Pose2d(new Translation2d( 15.813826560974121, 4.350637844085693 ), new Rotation2d(-Math.PI + 1.047)))),
        
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

