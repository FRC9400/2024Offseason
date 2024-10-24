package frc.robot.autons;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {

    public static Command TestAuto(Swerve swerve, Superstructure superstructure, String startingPos, ChoreoTrajectory traj){
        return Commands.sequence(
            resetGyroAuto(swerve, "startingPos"),
            resetPoseAuto(traj, swerve),
            new InstantCommand(() -> swerve.runChoreoTrajStandard(traj))

        );
    }

    public static Command preloadMid(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            new InstantCommand(() -> superstructure.requestAutoShootSubwooferM())
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
        ChoreoTrajectory trajA = Choreo.getTrajectory("FourNoteMidA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("FourNoteMidB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("FourNoteMidC");
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            resetPoseAuto(trajA, swerve),
            new InstantCommand(() -> superstructure.requestAutoShootSubwooferM()),
            intakeIn(swerve, superstructure, trajA),
            new InstantCommand(() -> superstructure.requestAutoShootMidNote()),
            intakeIn(swerve, superstructure, trajB),
            requestAmpShoot(superstructure),
            intakeIn(swerve, superstructure, trajC),
            requestSourceShoot(superstructure)
        );
    }

    public static Command fourNoteAmp(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("FourNoteAmpA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("FourNoteAmpB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("FourNoteAmpC");
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            resetPoseAuto(trajA, swerve),
            requestAmpSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA),
            requestAmpShoot(superstructure),
            intakeIn(swerve, superstructure, trajB),
            new InstantCommand(() -> superstructure.requestAutoShootMidNote()),
            intakeIn(swerve, superstructure, trajC),
            requestSourceShoot(superstructure)
        );
    }

    public static Command fourNoteSource(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("FourNoteSourceA");
        ChoreoTrajectory trajB = Choreo.getTrajectory("FourNoteSourceB");
        ChoreoTrajectory trajC = Choreo.getTrajectory("FourNoteSourceC");
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            resetPoseAuto(trajA, swerve),
            requestSourceSubwooferShoot(superstructure),
            intakeIn(swerve, superstructure, trajA),
            requestSourceShoot(superstructure),
            intakeIn(swerve, superstructure, trajB),
            new InstantCommand(() -> superstructure.requestAutoShootMidNote()),
            intakeIn(swerve, superstructure, trajC),
            requestAmpShoot(superstructure)
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
        if (startingPos.equals("mid")) {
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(0));
        }
        else if (startingPos.equals("source")) {
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -60 : -60));
        }
        else if (startingPos.equals("amp")){
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 60 : - 60));
        }

        return Commands.none();
    }

    public static Command intakeIn(Swerve swerve, Superstructure superstructure, ChoreoTrajectory traj) {
        return Commands.runOnce(() -> superstructure.requestAutoIntake())
            .deadlineWith(new InstantCommand(() -> swerve.runChoreoTrajStandard(traj)));
    }

    public static Command shoot(Swerve swerve, Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestPreShoot());
    }

    public static Command idleCommand(Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestAutoIdle());
    }

    public static Command requestAmpShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoShootRight();
            } else {
                superstructure.requestAutoShootLeft();
            }
        });
    }

    public static Command requestSourceShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoShootLeft();
            } else {
                superstructure.requestAutoShootRight();
            }
        });
    }

    public static Command requestAmpSubwooferShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoShootSubwooferR();
            } else {
                superstructure.requestAutoShootSubwooferL();
            }
        });
    }

    public static Command requestSourceSubwooferShoot(Superstructure superstructure) {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoShootSubwooferL();
            } else {
                superstructure.requestAutoShootSubwooferR();
            }
        });
    }
    
    
    }
    

