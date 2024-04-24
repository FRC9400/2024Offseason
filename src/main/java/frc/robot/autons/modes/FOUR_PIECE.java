package frc.robot.autons.modes;

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

public class FOUR_PIECE extends SequentialCommandGroup{
    //private final double startingAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0;
    private final Pose2d startingPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? new Pose2d(new Translation2d( 1.390610694885254, 5.51975345611572 ), new Rotation2d(0)) : new Pose2d(new Translation2d( 14.91, 5.558 ), new Rotation2d(3.14));
    //private final Pose2d startingPose = new Pose2d(new Translation2d( 1.390610694885254, 5.51975345611572 ), new Rotation2d(0));
    private final PathPlannerPath B =  PathPlannerPath.fromChoreoTrajectory("4pB");
    private final PathPlannerPath Bback = PathPlannerPath.fromChoreoTrajectory("4pBback");

    private final PathPlannerPath C =  PathPlannerPath.fromChoreoTrajectory("4pC");
    private final PathPlannerPath Cback =  PathPlannerPath.fromChoreoTrajectory("4pCback");

    private final PathPlannerPath A = PathPlannerPath.fromChoreoTrajectory("4pA");
    private final PathPlannerPath Aback =  PathPlannerPath.fromChoreoTrajectory("4pAback");

    public FOUR_PIECE(Swerve swerve, Superstructure superstructure){

        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0)),
        new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d( 14.91, 5.558 ), new Rotation2d(3.14)))),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(B),
        new WaitCommand(0.75),
        AutoBuilder.followPath(Bback),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(C),
        new WaitCommand(1),
        AutoBuilder.followPath(Cback),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
        AutoBuilder.followPath(A),
        new WaitCommand(1),
        AutoBuilder.followPath(Aback),
        new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
        new WaitCommand(1)
        );
    }
}

