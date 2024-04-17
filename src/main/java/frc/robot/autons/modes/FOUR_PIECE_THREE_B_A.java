package frc.robot.autons.modes;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class FOUR_PIECE_THREE_B_A extends SequentialCommandGroup{
    private final double startingAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0;
    private final Pose2d startingPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? new Pose2d(new Translation2d( 1.574845314025879, 5.54275345611572 ), new Rotation2d(0)) : new Pose2d(new Translation2d( 14.96, 5.542 ), new Rotation2d(3.14));
    //7.7 inches???? wtf bro
    private final PathPlannerPath Three = PathPlannerPath.fromChoreoTrajectory("4pRaceThree");
    private final PathPlannerPath ThreeBack = PathPlannerPath.fromChoreoTrajectory("4pRaceThreeback");
    private final PathPlannerPath B = PathPlannerPath.fromChoreoTrajectory("4pRaceB");
    private final PathPlannerPath Bback = PathPlannerPath.fromChoreoTrajectory("4pBback");
    private final PathPlannerPath A = PathPlannerPath.fromChoreoTrajectory("4pA");
    private final PathPlannerPath Aback =  PathPlannerPath.fromChoreoTrajectory("4pAback");


    public FOUR_PIECE_THREE_B_A(Swerve swerve, Superstructure superstructure){

        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.setGyroStartingPosition(startingAngle)),
        new InstantCommand(() -> swerve.resetPose(startingPose)),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1),
        //new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(Three),
        new WaitCommand(0.75),
        AutoBuilder.followPath(ThreeBack),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1),
        //new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(B),
        new WaitCommand(1),
        AutoBuilder.followPath(Bback),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1),
        //new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(A),
        new WaitCommand(1),
        AutoBuilder.followPath(Aback),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1)
        );
    }
}


