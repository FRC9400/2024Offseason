package frc.robot.autons.modes;

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
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }

    public static Command preloadAmp(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }

    public static Command preloadSource(Swerve swerve, Superstructure superstructure){
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }

    public static Command fourNoteMid(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("X");
        ChoreoTrajectory trajB = Choreo.getTrajectory("X");
        ChoreoTrajectory trajC = Choreo.getTrajectory("X");
        return Commands.sequence(
            resetGyroAuto(swerve, "mid"),
            resetPoseAuto(trajA, swerve),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajA),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajB),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajC),
            shoot(swerve, superstructure)
        );
    }

    public static Command fourNoteAmp(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("X");
        ChoreoTrajectory trajB = Choreo.getTrajectory("X");
        ChoreoTrajectory trajC = Choreo.getTrajectory("X");
        return Commands.sequence(
            resetGyroAuto(swerve, "amp"),
            resetPoseAuto(trajA, swerve),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajA),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajB),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajC),
            shoot(swerve, superstructure)
        );
    }

    public static Command fourNoteSource(Swerve swerve, Superstructure superstructure){
        ChoreoTrajectory trajA = Choreo.getTrajectory("X");
        ChoreoTrajectory trajB = Choreo.getTrajectory("X");
        ChoreoTrajectory trajC = Choreo.getTrajectory("X");
        return Commands.sequence(
            resetGyroAuto(swerve, "source"),
            resetPoseAuto(trajA, swerve),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajA),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajB),
            shoot(swerve, superstructure),
            intakeIn(swerve, superstructure, trajC),
            shoot(swerve, superstructure)
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
        return Commands.runOnce(() -> superstructure.requestIntake())
            .deadlineWith(new InstantCommand(() -> swerve.runChoreoTrajStandard(traj)));
    }

    public static Command shoot(Swerve swerve, Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestPreShoot());
    }
    
    }
    


