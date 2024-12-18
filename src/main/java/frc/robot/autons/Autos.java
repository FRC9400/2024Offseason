package frc.robot.autons;

import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {

    public static Command twoNoteMid(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory traj = Choreo.getTrajectory("MidA2");
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            resetPoseAuto(traj, swerve),
            requestMidSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, traj),
            requestMidShoot(superstructure),
            idleCommand(swerve, superstructure),
            finishGyro(swerve, "mid"));
    }

    public static Command twoNoteAmp(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory traj = Choreo.getTrajectory("AmpA");
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            resetPoseAuto(traj, swerve),
            requestAmpSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, traj),
            requestAmpShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }
    public static Command twoNoteSource(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory traj = Choreo.getTrajectory("SourceA");
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            resetPoseAuto(traj, swerve),
            requestSourceSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, traj),
            requestAmpShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }

    public static Command preloadMid(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            requestMidSubwooferShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }

    public static Command preloadAmp(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            requestAmpSubwooferShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }

    public static Command preloadSource(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            requestSourceSubwooferShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }

    public static Command fourNoteMid(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("MidA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("AmpB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("AmpC");
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            resetPoseAuto(trajA, swerve),
            requestMidSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA),
            requestAmpShoot(superstructure),
            intakeIn(swerve, superstructure, trajB),
            requestMidShoot(superstructure),
            intakeIn(swerve, superstructure, trajC),
            requestSourceShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }

    public static Command fourNoteAmp(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("AmpA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("AmpB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("AmpC");
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            resetPoseAuto(trajA, swerve),
            requestAmpSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA),
            requestAmpShoot(superstructure),
            intakeIn(swerve, superstructure, trajB),
            requestMidShoot(superstructure),
            intakeIn(swerve, superstructure, trajC),
            requestSourceShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }

    public static Command fourNoteSource(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("SourceA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("SourceB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("SourceC");
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            resetPoseAuto(trajA, swerve),
            requestSourceSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA),
            requestSourceShoot(superstructure),
            intakeIn(swerve, superstructure, trajB),
            requestMidShoot(superstructure),
            intakeIn(swerve, superstructure, trajC),
            requestAmpShoot(superstructure),
            idleCommand(swerve, superstructure)
        );
    }
    

   public static Command resetPoseAuto(ChoreoTrajectory trajectory, Swerve swerve) {
      return Commands.runOnce(
          () ->
          swerve.resetPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? trajectory.getInitialPose()
                : trajectory.getFlippedInitialPose()));
    }

    public static Command resetGyroAuto(Swerve swerve, String startingPos) {
        if (startingPos.equals("mid")){
            return Commands.runOnce(() -> swerve. setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 180: 180));
        }
        else if (startingPos.equals("source")) {
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 120 : -120));
        }
        else if (startingPos.equals("amp")){
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -120 : 120));
        }

        return Commands.none();
    }

    public static Command intakeIn(Swerve swerve, Superstructure superstructure, ChoreoTrajectory traj) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.NOTE;
        };
        return new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.requestIntake()),
            swerve.runChoreoTrajStandard(traj),
            new WaitUntilCommand(bool).withTimeout(3)
            );
    }

    public static Command idleCommand(Swerve swerve, Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestIdle());
    }

    public static Command finishGyro(Swerve swerve, String startingPos) {
        if (startingPos.equals("mid")){
            return Commands.runOnce(() -> swerve. setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 180:0));
        }
        else if (startingPos.equals("source")) {
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -120 : 120));
        }
        else if (startingPos.equals("amp")){
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 120 : -120));
        }

        return Commands.none();
    }
    

    public static Command requestMidShoot(Superstructure superstructure){
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> superstructure.requestAutoPreShoot(AutoConstants.VelM, AutoConstants.RatioM, AutoConstants.DegM))
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestMidSubwooferShoot(Superstructure superstructure){
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> superstructure.requestAutoPreShoot(AutoConstants.subwooferVelM, AutoConstants.subwooferRatioM, AutoConstants.subwooferDegM))
            .andThen(new WaitUntilCommand(bool));

    }

    public static Command requestAmpShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.VelR, AutoConstants.RatioR, AutoConstants.DegR);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.VelL, AutoConstants.RatioL, AutoConstants.DegL);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestSourceShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.VelL, AutoConstants.RatioL, AutoConstants.DegL);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.VelR, AutoConstants.RatioR, AutoConstants.DegR);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestAmpSubwooferShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelR, AutoConstants.subwooferRatioR, AutoConstants.subwooferDegR);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelL, AutoConstants.subwooferRatioL, AutoConstants.subwooferDegL);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestSourceSubwooferShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelL, AutoConstants.subwooferRatioL, AutoConstants.subwooferDegL);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelR, AutoConstants.subwooferRatioR, AutoConstants.subwooferDegR);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }
    }