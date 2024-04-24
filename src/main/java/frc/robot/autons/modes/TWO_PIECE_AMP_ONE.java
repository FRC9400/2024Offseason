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

public class TWO_PIECE_AMP_ONE extends SequentialCommandGroup{

    private final Pose2d startingPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? new Pose2d(new Translation2d( 0.6889407634735107, 6.747675895690918 ), new Rotation2d(1.047047697709670)) : new Pose2d(new Translation2d( 15.716826560974121, 6.728185653686523 ), new Rotation2d(Math.PI - 1.047));

    private final double startingAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? 60 : -60;
    private final PathPlannerPath forward =  PathPlannerPath.fromChoreoTrajectory("2pOne");
    private final PathPlannerPath backward = PathPlannerPath.fromChoreoTrajectory("2pOneback");
    
    public TWO_PIECE_AMP_ONE(Swerve swerve, Superstructure superstructure){
        
        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.setGyroStartingPosition( DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? 60 : -60)),
        new InstantCommand(() -> swerve.resetPose(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue? new Pose2d(new Translation2d( 0.6889407634735107, 6.747675895690918 ), new Rotation2d(1.047047697709670)) : new Pose2d(new Translation2d( 15.716826560974121, 6.728185653686523 ), new Rotation2d(Math.PI - 1.047)))),
        
       // new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_LEFT)),
        new WaitCommand(1.25),
        //new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(forward),
        new WaitCommand(2.5),
        AutoBuilder.followPath(backward)
       // new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_LEFT))
        );
        
    }
}

