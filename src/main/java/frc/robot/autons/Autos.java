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
        ChoreoTrajectory traj = Choreo.getTrajectory("MidA");
         BooleanSupplier boolIntake = () -> {
            return superstructure.getState() == SuperstructureStates.NOTE;
         };
         BooleanSupplier boolShoot = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT;
         };
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            resetPoseAuto(traj, swerve),
            requestMidSubwooferShoot(superstructure),
            new WaitUntilCommand(boolShoot),
            intake(swerve, superstructure),
            intakeIn(swerve, superstructure, traj),
            new WaitUntilCommand(boolIntake),
            requestMidShoot(superstructure)
        );
    }

    public static Command twoNoteAmp(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory traj = Choreo.getTrajectory("AmpA");
        BooleanSupplier boolIntake = () -> {
            return superstructure.getState() == SuperstructureStates.NOTE;
        };
        BooleanSupplier boolShoot = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT;
         };
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            resetPoseAuto(traj, swerve),
            requestAmpSubwooferShoot(superstructure),
            new WaitUntilCommand(boolShoot),
            intake(swerve, superstructure),
            intakeIn(swerve, superstructure, traj),
            new WaitUntilCommand(boolIntake),
            requestAmpShoot(superstructure)
        );
    }
    public static Command twoNoteSource(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory traj = Choreo.getTrajectory("SourceA");
        BooleanSupplier boolIntake = () -> {
            return superstructure.getState() == SuperstructureStates.NOTE;
        };
        BooleanSupplier boolShoot = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT;
         };
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            resetPoseAuto(traj, swerve),
            requestSourceSubwooferShoot(superstructure),
            new WaitUntilCommand(boolShoot),
            intake(swerve, superstructure),
            intakeIn(swerve, superstructure, traj),
            new WaitUntilCommand(boolIntake),
            requestAmpShoot(superstructure)
        );
    }

    public static Command preloadMid(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            requestMidSubwooferShoot(superstructure)
        );
    }

    public static Command preloadAmp(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            requestAmpSubwooferShoot(superstructure)
        );
    }

    public static Command preloadSource(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            requestSourceSubwooferShoot(superstructure)
        );
    }

    public static Command fourNoteMid(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("MidA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("MidB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("MidC");
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            resetPoseAuto(trajA, swerve),
            requestMidSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA),
          //  new InstantCommand(() -> superstructure.requestAutoShootMidNote()),
            intakeIn(swerve, superstructure, trajB),
            requestAmpShoot(superstructure),
            intakeIn(swerve, superstructure, trajC),
            requestSourceShoot(superstructure)
        );
    }

    public static Command fourNoteAmp(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("tuneX");
        ChoreoTrajectory trajB = Choreo.getTrajectory("AmpB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("AmpC");
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            resetPoseAuto(trajA, swerve),
           // requestAmpSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA)
            
         //   requestAmpShoot(superstructure),
          // intakeIn(swerve, superstructure, trajB)
          //  new InstantCommand(() -> superstructure.requestAutoShootMidNote()),
           // intakeIn(swerve, superstructure, trajC)          //  requestSourceShoot(superstructure)
        );
    }

    public static Command fourNoteSource(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("tuneY");
        ChoreoTrajectory trajB = Choreo.getTrajectory("SourceB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("SourceC");
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            resetPoseAuto(trajA, swerve),
         //   requestSourceSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA)
         //   requestSourceShoot(superstructure),
         //   intakeIn(swerve, superstructure, trajB),
        //    new InstantCommand(() -> superstructure.requestAutoShootMidNote()),
         //   intakeIn(swerve, superstructure, trajC),
         //   requestAmpShoot(superstructure)
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
            return Commands.runOnce(() -> swerve. setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 180));
        }
        else if (startingPos.equals("source")) {
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 120 : -120));
        }
        else if (startingPos.equals("amp")){
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -120 : 120));
        }

        return Commands.none();
    }

    public static Command intakeIn2(Swerve swerve, Superstructure superstructure, ChoreoTrajectory traj) {
        return Commands.run(() -> superstructure.requestIntake())
            .deadlineWith(swerve.runChoreoTrajStandard(traj)); //Changed this from deadline to parallel
    }

    
    public static Command intakeIn(Swerve swerve, Superstructure superstructure, ChoreoTrajectory traj) {
        return swerve.runChoreoTrajStandard(traj); //Changed this from deadline to parallel
    }

    public static Command intake(Swerve swerve, Superstructure superstructure) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitCommand(0.5));
    }

    public static Command idleCommand(Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestIdle());
    }

    public static Command requestMidShoot(Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestPreShoot(AutoConstants.VelM, AutoConstants.RatioM, AutoConstants.DegM));
    }

    public static Command requestMidSubwooferShoot(Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestPreShoot(AutoConstants.subwooferVelM, AutoConstants.subwooferRatioM, AutoConstants.subwooferDegM));
    }

    public static Command requestAmpShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestPreShoot(AutoConstants.VelR, AutoConstants.RatioR, AutoConstants.DegR);
            } else {
                superstructure.requestPreShoot(AutoConstants.VelL, AutoConstants.RatioL, AutoConstants.DegL);
            }
        });
    }

    public static Command requestSourceShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestPreShoot(AutoConstants.VelL, AutoConstants.RatioL, AutoConstants.DegL);
            } else {
                superstructure.requestPreShoot(AutoConstants.VelR, AutoConstants.RatioR, AutoConstants.DegR);
            }
        });
    }

    public static Command requestAmpSubwooferShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestPreShoot(AutoConstants.subwooferVelR, AutoConstants.subwooferRatioR, AutoConstants.subwooferDegR);
            } else {
                superstructure.requestPreShoot(AutoConstants.subwooferVelL, AutoConstants.subwooferRatioL, AutoConstants.subwooferDegL);
            }
        });
    }

    public static Command requestSourceSubwooferShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestPreShoot(AutoConstants.subwooferVelL, AutoConstants.subwooferRatioL, AutoConstants.subwooferDegL);
            } else {
                superstructure.requestPreShoot(AutoConstants.subwooferVelR, AutoConstants.subwooferRatioR, AutoConstants.subwooferDegR);
            }
        });
    }
    }
    

