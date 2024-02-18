package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;
    private TalonFXConfiguration leftShooterConfigs;
    private TalonFXConfiguration rightShooterConfigs;
    private TalonFXConfigurator leftShooterConfigurator;
    private TalonFXConfigurator rightShooterConfigurator;

    
    private double leftShooterSetpointMPS = 0;
    private double speedRatio = 0;
    private double rightShooterSetpointMPS = 0;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequesetVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private MotionMagicVelocityVoltage shootRequestMotionMagic = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

    LoggedTunableNumber speedRatioTune = new LoggedTunableNumber("Shooter/speedRatio", 0.7);
    LoggedTunableNumber leftShooterSpeedMPS = new LoggedTunableNumber("Shooter/shooterSpeedMPS", 5);

    LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("Shooter/kMotionCruiseVelocity", 0.0);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber("Shooter/kMotionAcceleration", 0.0);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Shooter/kMotionJerk", 0.0);

    public ShooterIOTalonFX() {
        leftShooter = new TalonFX(canIDConstants.leftShooterMotor);
        rightShooter = new TalonFX(canIDConstants.rightShooterMotor);
        TalonFXConfiguration leftShooterConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightShooterConfigs = new TalonFXConfiguration();
        TalonFXConfigurator leftShooterConfigurator = leftShooter.getConfigurator();
        TalonFXConfigurator rightShooterConfigurator = rightShooter.getConfigurator();
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.currentAmps = new double[] { leftShooter.getStatorCurrent().getValue(),
                rightShooter.getStatorCurrent().getValue() };
        inputs.tempFahrenheit = new double[] { leftShooter.getDeviceTemp().getValue(),
                rightShooter.getDeviceTemp().getValue() };
        inputs.shooterSpeedRPS = new double[] { leftShooter.getRotorVelocity().getValue(),
                rightShooter.getRotorVelocity().getValue() };
        inputs.shooterSpeedMPS = new double[] {Conversions.RPStoMPS(leftShooter.getRotorVelocity().getValue(), shooterConstants.wheelCircumferenceMeters, 1), Conversions.RPStoMPS(rightShooter.getRotorVelocity().getValue(), shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsRPS = new double[] {Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1), Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsMPS = new double[] {leftShooterSetpointMPS, rightShooterSetpointMPS};

    }

    public void updateTunableNumbers() {
        if  (kP.hasChanged(kP.hashCode()) ||
                kD.hasChanged(kD.hashCode()) ||
                kS.hasChanged(kS.hashCode()) ||
                kV.hasChanged(kV.hashCode()) ||
                kMotionCruiseVelocity.hasChanged((kMotionCruiseVelocity.hashCode())) ||
                kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode())  ||
                kMotionJerk.hasChanged(kMotionJerk.hashCode())||
                speedRatioTune.hasChanged(speedRatioTune.hashCode())||
                leftShooterSpeedMPS.hasChanged(leftShooterSpeedMPS.hashCode()) 
                ){
            shooterConfiguration();
        }
    }

    public void setVoltage(double volts) {
        leftShooter.setControl(shootRequestVoltage.withOutput(volts));
        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void setVelocity(){
        leftShooter.setControl(leftShootRequesetVelocity.withVelocity(Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1)));
        rightShooter.setControl(rightShootRequestVelocity.withVelocity(Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1)));
        //leftShooter.setControl(shootRequestMotionMagic.withVelocity(velocityRPS));
    }

    public void zeroVelocity(){
        leftShooter.setControl(leftShootRequesetVelocity.withVelocity(0));
        rightShooter.setControl(rightShootRequestVelocity.withVelocity(0));

    }

    public void shooterConfiguration() {
        var leftShooterMotorConfigs = leftShooterConfigs.MotorOutput;
        var rightShooterMotorConfigs = rightShooterConfigs.MotorOutput;
        leftShooterMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftShooterMotorConfigs.PeakForwardDutyCycle = 1.0;
        leftShooterMotorConfigs.PeakReverseDutyCycle = -1.0;
        leftShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;
        leftShooterMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;

        rightShooterMotorConfigs.PeakForwardDutyCycle = 1.0;
        rightShooterMotorConfigs.PeakReverseDutyCycle = -1.0;
        rightShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var leftShooterCurrentConfigs = leftShooterConfigs.CurrentLimits;
        leftShooterCurrentConfigs.StatorCurrentLimitEnable = false;
        leftShooterCurrentConfigs.StatorCurrentLimit = 0;

        var rightShooterCurrentConfigs = rightShooterConfigs.CurrentLimits;
        rightShooterCurrentConfigs.StatorCurrentLimitEnable = false;
        leftShooterCurrentConfigs.StatorCurrentLimit = 0;

        var leftSlot0Configs = leftShooterConfigs.Slot0;
        leftSlot0Configs.kP = kP.get();
        leftSlot0Configs.kI = 0.0;
        leftSlot0Configs.kD = kD.get();
        leftSlot0Configs.kS = kS.get();
        leftSlot0Configs.kV = kV.get();

        var rightSlot0Configs = rightShooterConfigs.Slot0;
        rightSlot0Configs.kP = kP.get();
        rightSlot0Configs.kI = 0.0;
        rightSlot0Configs.kD = kD.get();
        rightSlot0Configs.kS = kS.get();
        rightSlot0Configs.kV = kV.get();

        var leftShooterMotionMagicConfigs = leftShooterConfigs.MotionMagic;
        leftShooterMotionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        leftShooterMotionMagicConfigs.MotionMagicAcceleration = 0.0;
        leftShooterMotionMagicConfigs.MotionMagicJerk = 0.0;

        leftShooterConfigurator.apply(leftShooterConfigs);
        rightShooterConfigurator.apply(rightShooterConfigs);

        leftShooterSetpointMPS = leftShooterSpeedMPS.get();
        speedRatio = speedRatioTune.get();
        rightShooterSetpointMPS = leftShooterSetpointMPS * speedRatio;
    }
}
