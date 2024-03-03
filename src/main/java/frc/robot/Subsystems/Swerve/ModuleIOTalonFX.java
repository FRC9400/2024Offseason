package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.commons.LoggedTunableNumber;
import frc.commons.Conversions;
import frc.robot.Constants.swerveConstants;

public class ModuleIOTalonFX implements ModuleIO{
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;
    private final Rotation2d CANcoderOffset;
    private final InvertedValue driveInvert;
    private final InvertedValue steerInvert;
    private final SensorDirectionValue CANcoderInvert;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;
    private CANcoderConfigurator angleEncoderConfigurator;
    private TalonFXConfigurator driveConfigurator;
    private TalonFXConfigurator steerConfigurator;

    private DutyCycleOut driveRequest;
    private PositionDutyCycle steerRequest;
    private VelocityVoltage velocityVoltageRequest;
    private VoltageOut voltageRequest;


    LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/kP", 0);
    LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/kD", 0);
    LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/kS", 0);
    LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/kV", 0);

    LoggedTunableNumber steerkP = new LoggedTunableNumber("Steer/kP", 0);
    LoggedTunableNumber steerkD = new LoggedTunableNumber("Steer/kD", 0);
    LoggedTunableNumber steerkS = new LoggedTunableNumber("Steer/kS", 0);
    LoggedTunableNumber steerkV = new LoggedTunableNumber("Steer/kV", 0);

    LoggedTunableNumber voltage = new LoggedTunableNumber("Drive/Voltage", 0);



    public ModuleIOTalonFX(int driveID, int steerID, int CANcoderID, Rotation2d CANcoderOffset, InvertedValue driveInvert, InvertedValue steerInvert, SensorDirectionValue CANcoderInvert){
        driveMotor = new TalonFX(driveID);
        steerMotor = new TalonFX(steerID);
        angleEncoder = new CANcoder(CANcoderID);
        this.CANcoderOffset = CANcoderOffset;
        this.driveInvert = driveInvert;
        this.steerInvert = steerInvert;
        this.CANcoderInvert = CANcoderInvert;
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();
        driveConfigurator = driveMotor.getConfigurator();
        steerConfigurator = steerMotor.getConfigurator();
        angleEncoderConfigurator = angleEncoder.getConfigurator();

        driveRequest = new DutyCycleOut(0).withEnableFOC(true);
        steerRequest = new PositionDutyCycle(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        voltageRequest = new VoltageOut(0).withEnableFOC(true);


        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = driveInvert; 
        driveMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
        driveMotorOutputConfigs.PeakReverseDutyCycle = -1.0;

        var driveFeedbackConfigs = driveConfigs.Feedback;
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setPosition(0);

        var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
        driveCurrentLimitConfigs.StatorCurrentLimitEnable = false;
        driveCurrentLimitConfigs.StatorCurrentLimit = 0;

        var driveSlot0Configs = driveConfigs.Slot0;
        driveSlot0Configs.kP = drivekP.get();
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = drivekD.get();
        driveSlot0Configs.kS = drivekS.get();
        driveSlot0Configs.kV = drivekV.get();

        //STEER

        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = steerInvert;

        var steerFeedbackConfigs = steerConfigs.Feedback;
        steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        var steerSlot0Configs = steerConfigs.Slot0;
        steerSlot0Configs.kP = steerkP.get();
        steerSlot0Configs.kI = 0.0;
        steerSlot0Configs.kD = steerkD.get();
        steerSlot0Configs.kS = steerkS.get();
        steerSlot0Configs.kV = steerkV.get();

        var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
        steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        steerCurrentLimitConfigs.StatorCurrentLimit = 50;

        // CANcoder
        var magnetSensorConfigs = angleEncoderConfigs.MagnetSensor;
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.MagnetOffset = 0;
        magnetSensorConfigs.SensorDirection = CANcoderInvert;


        driveConfigurator.apply(driveConfigs);
        steerConfigurator.apply(steerConfigs);
        angleEncoderConfigurator.apply(angleEncoderConfigs);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){
        inputs.driveVelocityMetersPerSec = Conversions.RPStoMPS(driveMotor.getRotorVelocity().getValue(), swerveConstants.moduleConstants.wheelCircumferenceMeters, swerveConstants.moduleConstants.driveGearRatio);
        inputs.driveAppliedVolts = driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValue();
        inputs.driveTempCelcius = driveMotor.getDeviceTemp().getValue();
        inputs.driveDistanceMeters = Conversions.RotationsToMeters(driveMotor.getRotorPosition().getValue(), swerveConstants.moduleConstants.wheelCircumferenceMeters, swerveConstants.moduleConstants.driveGearRatio);
        inputs.driveOutputPercent = driveMotor.get();
        inputs.rawDriveRPS = driveMotor.getRotorVelocity().getValue();

        inputs.moduleAngleRads = Units.degreesToRadians(Conversions.RotationsToDegrees(steerMotor.getPosition().getValue(), swerveConstants.moduleConstants.steerGearRatio));
        inputs.moduleAngleDegs = Conversions.RotationsToDegrees(steerMotor.getPosition().getValue(), swerveConstants.moduleConstants.steerGearRatio);
        inputs.rawAbsolutePositionRotations = angleEncoder.getAbsolutePosition().getValue();
        inputs.absolutePositionRadians = angleEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
        inputs.absolutePositionDegrees = angleEncoder.getAbsolutePosition().getValue() * 360;
        inputs.turnAppliedVolts = steerMotor.getDutyCycle().getValue() * steerMotor.getSupplyVoltage().getValue();
        inputs.turnCurrentAmps = steerMotor.getStatorCurrent().getValue();
        inputs.turnTempCelcius = steerMotor.getDeviceTemp().getValue();
    }

    @Override
    public void updateTunableNumbers(){
        if (
        drivekD.hasChanged(drivekD.hashCode()) ||
        drivekS.hasChanged(drivekS.hashCode()) ||
        drivekP.hasChanged(drivekP.hashCode()) ||
        drivekV.hasChanged(drivekV.hashCode())
        ) {
        var driveSlot0Configs = new Slot0Configs();
        driveSlot0Configs.kP = drivekP.get();
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = drivekD.get();
        driveSlot0Configs.kS = drivekS.get();
        driveSlot0Configs.kV = drivekV.get();

        driveConfigurator.apply(driveSlot0Configs);
        }

        if (
        steerkD.hasChanged(steerkD.hashCode()) ||
        steerkS.hasChanged(steerkS.hashCode()) ||
        steerkP.hasChanged(steerkP.hashCode()) ||
        steerkV.hasChanged(steerkV.hashCode())
        ){
        var steerSlot0Configs = new Slot0Configs();
        steerSlot0Configs.kP = steerkP.get();
        steerSlot0Configs.kI = 0.0;
        steerSlot0Configs.kD = steerkD.get();
        steerSlot0Configs.kS = steerkS.get();
        steerSlot0Configs.kV = steerkV.get();

        steerConfigurator.apply(steerSlot0Configs);
        }
    }

    public void setDesiredState(SwerveModuleState optimizedDesiredStates){
        double drivePercentOut = optimizedDesiredStates.speedMetersPerSecond / (swerveConstants.moduleConstants.maxSpeed);
        double angleDeg = optimizedDesiredStates.angle.getDegrees();

        setDrivePercent(drivePercentOut);
        setTurnAngle(angleDeg);
    }

    public void testDriveVoltage(){
        driveMotor.setControl(voltageRequest.withOutput(voltage.get()));
    }

    public void setDriveVoltage(double setVoltage){
        driveMotor.setControl(voltageRequest.withOutput(setVoltage));
    }

    public void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {
        velocityVoltageRequest.Velocity = Conversions.MPStoRPS(velocityMetersPerSecond, swerveConstants.moduleConstants.wheelCircumferenceMeters, swerveConstants.moduleConstants.driveGearRatio);
        driveMotor.setControl(velocityVoltageRequest);
    }
}