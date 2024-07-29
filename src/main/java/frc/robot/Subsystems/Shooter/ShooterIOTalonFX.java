package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX leftShooter = new TalonFX(canIDConstants.leftShooterMotor);
    private final TalonFX rightShooter = new TalonFX(canIDConstants.rightShooterMotor);
    private final TalonFX leftArm = new TalonFX(canIDConstants.leftArmMotor);
    private final TalonFX rightArm = new TalonFX(canIDConstants.rightArmMotor);
    private final TalonFX ampRoller = new TalonFX(canIDConstants.ampRollerMotor);
    
    private TalonFXConfiguration leftShooterConfigs;
    private TalonFXConfiguration rightShooterConfigs;
    private TalonFXConfiguration leftArmConfigs;
    private TalonFXConfiguration ampRollerConfigs;

    private final StatusSignal<Double> leftShooterCurrent = leftShooter.getStatorCurrent();
    private final StatusSignal<Double> rightShooterCurrent = rightShooter.getStatorCurrent();
    private final StatusSignal<Double> leftShooterTemp = leftShooter.getDeviceTemp();
    private final StatusSignal<Double> rightShooterTemp = rightShooter.getDeviceTemp();
    private final StatusSignal<Double> leftShooterSpeedRPS = leftShooter.getRotorVelocity();
    private final StatusSignal<Double> rightShooterSpeedRPS = rightShooter.getRotorVelocity();

    private final StatusSignal<Double> leftArmCurrent = leftArm.getStatorCurrent();
    private final StatusSignal<Double> rightArmCurrent = rightArm.getStatorCurrent();
    private final StatusSignal<Double> leftArmTemp = leftArm.getDeviceTemp();
    private final StatusSignal<Double> rightArmTemp = rightArm.getDeviceTemp();
    private final StatusSignal<Double> leftArmPos = leftArm.getRotorPosition();
    private final StatusSignal<Double> rightArmPos = rightArm.getRotorPosition();
    private final StatusSignal<Double> leftArmRPS = leftArm.getRotorVelocity();
    private final StatusSignal<Double> rightArmRPS = rightArm.getRotorVelocity();

    private final StatusSignal<Double> ampRollerCurrent = ampRoller.getStatorCurrent();
    private final StatusSignal<Double> ampRollerTemp = ampRoller.getDeviceTemp();
    private final StatusSignal<Double> ampRollerRPS = ampRoller.getRotorVelocity();
    
    private double leftShooterSetpointMPS = 0;
    private double rightShooterSetpointMPS = 0;
    private double leftArmSetpointDegrees = 0;
    private double ampRollerSetpointVolts = 0;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

    private VoltageOut ampRollerRequestVoltage = new VoltageOut(0).withEnableFOC(true);

    private VoltageOut leftArmRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private MotionMagicVoltage leftArmMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    
    public ShooterIOTalonFX() {
        leftShooterConfigs = new TalonFXConfiguration();
        rightShooterConfigs = new TalonFXConfiguration();
        leftArmConfigs = new TalonFXConfiguration();
        ampRollerConfigs = new TalonFXConfiguration();

        var leftShooterMotorConfigs = leftShooterConfigs.MotorOutput;
        var rightShooterMotorConfigs = rightShooterConfigs.MotorOutput;
    
        leftShooterMotorConfigs.Inverted = shooterConstants.leftShooterInvert;
        leftShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        rightShooterMotorConfigs.Inverted = shooterConstants.rightShooterInvert;
        rightShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var leftShooterCurrentConfigs = leftShooterConfigs.CurrentLimits;
        leftShooterCurrentConfigs.StatorCurrentLimit = shooterConstants.shooterCurrentLimit;
        leftShooterCurrentConfigs.StatorCurrentLimitEnable = true;


        var rightShooterCurrentConfigs = rightShooterConfigs.CurrentLimits;
        rightShooterCurrentConfigs.StatorCurrentLimit = shooterConstants.shooterCurrentLimit;
        rightShooterCurrentConfigs.StatorCurrentLimitEnable = true;

        var leftShooterSlot0Configs = leftShooterConfigs.Slot0;
        leftShooterSlot0Configs.kP = 0.0;
        leftShooterSlot0Configs.kI = 0.0;
        leftShooterSlot0Configs.kD = 0.0;
        leftShooterSlot0Configs.kS = 0.0;
        leftShooterSlot0Configs.kV = 0.0;
        leftShooterSlot0Configs.kA = 0.0;

        var rightShooterSlot0Configs = rightShooterConfigs.Slot0;
        rightShooterSlot0Configs.kP = 0.0;
        rightShooterSlot0Configs.kI = 0.0;
        rightShooterSlot0Configs.kD = 0.0;
        rightShooterSlot0Configs.kS = 0.0;
        rightShooterSlot0Configs.kV = 0.0;
        rightShooterSlot0Configs.kA = 0.0;

        var leftArmMotorConfigs = leftArmConfigs.MotorOutput;

        leftArmMotorConfigs.Inverted = shooterConstants.leftArmInvert;
        leftArmMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        var leftArmCurrentConfigs = leftArmConfigs.CurrentLimits;

        leftArmCurrentConfigs.StatorCurrentLimit = shooterConstants.armCurrentLimit;
        leftArmCurrentConfigs.StatorCurrentLimitEnable = true;

        var leftArmSlot0Configs = leftArmConfigs.Slot0;
        leftArmSlot0Configs.kP = 0.0;
        leftArmSlot0Configs.kI = 0.0;
        leftArmSlot0Configs.kD = 0.0;
        leftArmSlot0Configs.kS = 0.0;
        leftArmSlot0Configs.kV = 0.0;
        leftArmSlot0Configs.kA = 0.0;
        leftArmSlot0Configs.kG = 0.0;
        leftArmSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = leftArmConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        motionMagicConfigs.MotionMagicAcceleration = 0.0;
        motionMagicConfigs.MotionMagicJerk = 0.0;

        var feedbackConfigs = leftArmConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        var ampRollerMotorConfigs = ampRollerConfigs.MotorOutput;
        ampRollerMotorConfigs.NeutralMode = NeutralModeValue.Coast;
        ampRollerMotorConfigs.Inverted = shooterConstants.ampRollerInvert;

        var ampRollerCurrentConfigs = ampRollerConfigs.CurrentLimits;
        ampRollerCurrentConfigs.StatorCurrentLimit = shooterConstants.ampRollerCurrentLimit;
        ampRollerCurrentConfigs.StatorCurrentLimitEnable = true;

        leftShooter.getConfigurator().apply(leftShooterConfigs);
        rightShooter.getConfigurator().apply(rightShooterConfigs);
        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));
        leftArm.getConfigurator().apply(leftArmConfigs);
        ampRoller.getConfigurator().apply(ampRollerConfigs);


        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
           leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftArmCurrent,
            rightArmCurrent,
            leftArmTemp,
            rightArmTemp,
            leftArmPos,
            rightArmPos,
            leftArmRPS,
            rightArmRPS,
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS
            );

        leftShooter.optimizeBusUtilization();
        rightShooter.optimizeBusUtilization();
    }

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftArmCurrent,
            rightArmCurrent,
            leftArmTemp,
            rightArmTemp,
            leftArmPos,
            rightArmPos,
            leftArmRPS,
            rightArmRPS,
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS
        );

        inputs.shooterAppliedVolts = shootRequestVoltage.Output;
        inputs.shooterCurrent = new double[] {leftShooterCurrent.getValue(),
                rightShooterCurrent.getValue() };
        inputs.shooterTemp = new double[] {leftShooterTemp.getValue(),
                rightShooterTemp.getValue() };
        inputs.shooterSpeedRPS = new double[] {leftShooterSpeedRPS.getValue(),
                rightShooterSpeedRPS.getValue() };
        inputs.shooterSpeedMPS = new double[] {Conversions.RPStoMPS(leftShooterSpeedRPS.getValue(), shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio), Conversions.RPStoMPS(rightShooterSpeedRPS.getValue(), shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio)};
        inputs.shooterSetpointsRPS = new double[] {Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio), Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio)};
        inputs.shooterSetpointsMPS = new double[] {leftShooterSetpointMPS, rightShooterSetpointMPS};

        inputs.armAppliedVolts = leftArmRequestVoltage.Output;
        inputs.armSetpointDeg = leftArmSetpointDegrees;
        inputs.armSetpointRot = Conversions.DegreesToRotations(leftArmSetpointDegrees, shooterConstants.armGearRatio);
        inputs.armPosRot = new double[] {leftArmPos.getValue(), rightArmPos.getValue()};
        inputs.armPosDeg = new double[] {Conversions.RotationsToDegrees(leftArmPos.getValue(), shooterConstants.armGearRatio), Conversions.RotationsToDegrees(rightArmPos.getValue(), shooterConstants.armGearRatio)};

        inputs.armCurrent = new double[] {leftArmCurrent.getValue(), rightArmCurrent.getValue()};
        inputs.armTemp = new double[] {leftArmTemp.getValue(), rightArmTemp.getValue()};
        inputs.armRPS = new double[] {leftArmRPS.getValue(), rightArmRPS.getValue()};

        inputs.ampRollerAppliedVolts = ampRollerRequestVoltage.Output;
        inputs.ampRollerTemp = ampRollerTemp.getValue();
        inputs.ampRollerCurrent = ampRollerCurrent.getValue();
        inputs.ampRollerRPS = ampRollerRPS.getValue();
    }


    public void requestShooterVoltage(double voltage) {
        leftShooter.setControl(shootRequestVoltage.withOutput(voltage));
        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void requestVelocity(double velocity, double ratio){
        leftShooterSetpointMPS = velocity;
        rightShooterSetpointMPS = velocity * ratio;
        leftShooter.setControl(leftShootRequestVelocity.withVelocity(Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio)));
        rightShooter.setControl(rightShootRequestVelocity.withVelocity(Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio)));
    }

    public void requestAmpRollerVoltage(double voltage) {
        ampRollerSetpointVolts = voltage;
        ampRoller.setControl(ampRollerRequestVoltage.withOutput(ampRollerSetpointVolts));
    }

    public void requestArmVoltage(double voltage) {
        leftArm.setControl(leftArmRequestVoltage.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees) {
        leftArmSetpointDegrees = angleDegrees;
        double leftArmSetpointRotations = Conversions.DegreesToRotations(angleDegrees, shooterConstants.armGearRatio);
        leftArm.setControl(leftArmMotionMagicRequest.withPosition(leftArmSetpointRotations));
    }

    public void zeroShooterVelocity(){
        leftShooterSetpointMPS = 0;
        rightShooterSetpointMPS = 0;
        leftShooter.setControl(leftShootRequestVelocity.withVelocity(0));
        rightShooter.setControl(rightShootRequestVelocity.withVelocity(0));
    }
}
