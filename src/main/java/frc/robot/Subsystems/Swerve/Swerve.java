 package frc.robot.Subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.kinematicsConstants;
import frc.robot.Subsystems.Swerve.GyroIO.GyroIOInputs;
import frc.robot.Subsystems.Swerve.ModuleIO.ModuleIOInputs;

public class Swerve extends SubsystemBase{
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    public final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private Pose2d poseRaw = new Pose2d();
    private Rotation2d lastGyroYaw = new Rotation2d();
    private final boolean fieldRelatve;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kinematicsConstants.FL, kinematicsConstants.FR, kinematicsConstants.BL,
        kinematicsConstants.BR);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(),
        getSwerveModulePositions()); 
    SwerveModuleState setpointModuleStates[] = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0,
            new Rotation2d()));
    
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };

    public Swerve() {
        gyroIO = new GyroIOPigeon2(0);

        moduleIOs[0] = new ModuleIOTalonFX(canIDConstants.driveMotor[0], canIDConstants.steerMotor[0], canIDConstants.CANcoder[0],swerveConstants.moduleConstants.CANcoderOffsets[0],
        swerveConstants.moduleConstants.driveMotorInverts[0], swerveConstants.moduleConstants.steerMotorInverts[0], swerveConstants.moduleConstants.CANcoderInverts[0]);

        moduleIOs[1] = new ModuleIOTalonFX(canIDConstants.driveMotor[1], canIDConstants.steerMotor[1], canIDConstants.CANcoder[1], swerveConstants.moduleConstants.CANcoderOffsets[1],
       swerveConstants.moduleConstants.driveMotorInverts[1], swerveConstants.moduleConstants.steerMotorInverts[1], swerveConstants.moduleConstants.CANcoderInverts[1]);

        moduleIOs[2] = new ModuleIOTalonFX(canIDConstants.driveMotor[2], canIDConstants.steerMotor[2], canIDConstants.CANcoder[2], swerveConstants.moduleConstants.CANcoderOffsets[2],
        swerveConstants.moduleConstants.driveMotorInverts[2], swerveConstants.moduleConstants.steerMotorInverts[2], swerveConstants.moduleConstants.CANcoderInverts[2]);

        moduleIOs[3] = new ModuleIOTalonFX(canIDConstants.driveMotor[3], canIDConstants.steerMotor[3], canIDConstants.CANcoder[3], swerveConstants.moduleConstants.CANcoderOffsets[3],
        swerveConstants.moduleConstants.driveMotorInverts[3], swerveConstants.moduleConstants.steerMotorInverts[3], swerveConstants.moduleConstants.CANcoderInverts[3]);
       
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].setDriveBrakeMode(true);
            moduleIOs[i].setTurnBrakeMode(false);
        }
        this.fieldRelatve = true;
      }


    @Override
    public void periodic(){
        updateOdometry();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++){
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Swerve/Module/ModuleNum[" + i + "]", moduleInputs[i]);
            moduleIOs[i].updateTunableNumbers();
        }

        logModuleStates("SwerveModuleStates/setpointStates", getSetpointStates());
        //logModuleStates("SwerveModuleStates/optimizedSetpointStates", getOptimizedSetPointStates());
        logModuleStates("SwerveModuleStates/MeasuredStates", getMeasuredStates());
        Logger.recordOutput("Odometry/PoseRaw", poseRaw);
        
    }

    /* Tuning is the worst thing ive done in my life */
    public void requestVoltage(boolean testVoltage){
        if(testVoltage){
            for(int i = 0; i < 4; i++){
                moduleIOs[i].testDriveVoltage();
            }
         }
        else{
            for(int i = 0; i < 4; i++){
                moduleIOs[i].setDriveVoltage(0);
            }
        }
    }

    public void requestPercent(double x_speed, double y_speed,double rot_speed, boolean fieldRelative){

        Rotation2d[] steerPositions = new Rotation2d[4];
        SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            steerPositions[i] = new Rotation2d(moduleInputs[i].moduleAngleRads);
        }
        Rotation2d gyroPositionDegrees = new Rotation2d(gyroInputs.positionRad);
        if (fieldRelative){
            desiredModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                rot_speed,
                gyroPositionDegrees));
            kinematics.desaturateWheelSpeeds(setpointModuleStates, swerveConstants.moduleConstants.maxSpeed);
            
            for (int i = 0; i < 4; i++) {
                setpointModuleStates[i] =  SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
                moduleIOs[i].setDesiredState(setpointModuleStates[i]);
            }
   
        }
    }

    public void requestVelocity(double velocityMetersPerSecond, boolean auto){
        for (int i = 0 ; i < 4; i++){
            moduleIOs[i].setDriveVelocity(velocityMetersPerSecond, auto);
        }
    }

    public void zeroWheels(){
        for(int i = 0; i < 4; i++){
            moduleIOs[i].resetToAbsolute();
        }
    }

    public void zeroGyro(){
        gyroIO.reset();
    }

    public void updateOdometry(){
        var gyroYaw = new Rotation2d(gyroInputs.positionRad);
        lastGyroYaw = gyroYaw;
        poseRaw = odometry.update(
                getRotation2d(),
                getSwerveModulePositions());
    }

    public Pose2d getPoseRaw(){
        return poseRaw;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroInputs.positionRad);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                    moduleInputs[i].driveDistanceMeters,
                    new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return modulePositions;
    }

    public SwerveModuleState[] getMeasuredStates(){
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            measuredStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return measuredStates;
    }
    public SwerveModuleState[] getSetpointStates(){
        return setpointModuleStates;
    }

    private void logModuleStates(String key, SwerveModuleState[] states) {
        List<Double> dataArray = new ArrayList<Double>();
        for (int i = 0; i < 4; i++) {
            dataArray.add(states[i].angle.getRadians());
            dataArray.add(states[i].speedMetersPerSecond);
        }
        Logger.recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
    }

}
