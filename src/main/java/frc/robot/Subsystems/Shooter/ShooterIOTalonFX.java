package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.commons.LoggedTunableNumber;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOTalonFX {
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;
    private TalonFXConfiguration leftShooterConfigs;
    private TalonFXConfiguration rightShooterConfigs;
    private TalonFXConfigurator leftShooterConfigurator;
    private TalonFXConfigurator rightShooterConfigurator;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private MotionMagicVelocityVoltage shootRequestMotionMagic = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

    LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("Shooter/kMotionCruiseVelocity", 0.0);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber("Shooter/kMotionAcceleration", 0.0);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Shooter/kMotionJerk", 0.0);

    public ShooterIOTalonFX(int leftShooterMotorID, int rightShooterMotorID) {
        leftShooter = new TalonFX(leftShooterMotorID);
        rightShooter = new TalonFX(rightShooterMotorID);
        TalonFXConfiguration leftShooterConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightShooterConfigs = new TalonFXConfiguration();
        TalonFXConfigurator leftShooterConfigurator = leftShooter.getConfigurator();
        TalonFXConfigurator rightShooterConfigurator = rightShooter.getConfigurator();
    }

    public void updateInputs(ShooterIOInputs shooterInputs) {
        shooterInputs.appliedVolts = shootRequestVoltage.Output;
        shooterInputs.currentAmps = new double[] { leftShooter.getStatorCurrent().getValue(),
                rightShooter.getStatorCurrent().getValue() };
        shooterInputs.tempFahrenheit = new double[] { leftShooter.getDeviceTemp().getValue(),
                rightShooter.getDeviceTemp().getValue() };
        shooterInputs.shooterSpeedRPS = new double[] { leftShooter.getRotorVelocity().getValue(),
                rightShooter.getRotorVelocity().getValue() };
    }

    public void updateTunableNumbers() {
        if  (kP.hasChanged(kP.hashCode()) ||
                kD.hasChanged(kD.hashCode()) ||
                kS.hasChanged(kS.hashCode()) ||
                kV.hasChanged(kV.hashCode()) ||
                kMotionCruiseVelocity.hasChanged((kMotionCruiseVelocity.hashCode())) ||
                kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode())  ||
                kMotionJerk.hasChanged(kMotionJerk.hashCode())){
            shooterConfiguration();
        }
    }

    public void setVoltage(double volts) {
        leftShooter.setControl(shootRequestVoltage.withOutput(volts));
    }

    public void setVelocity(double velocityRPS){
        leftShooter.setControl(shootRequestMotionMagic.withVelocity(velocityRPS));
    }

    public void shooterConfiguration() {
        var leftShooterMotorConfigs = leftShooterConfigs.MotorOutput;
        var rightShooterMotorConfigs = rightShooterConfigs.MotorOutput;
        leftShooterMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftShooterMotorConfigs.PeakForwardDutyCycle = 1.0;
        leftShooterMotorConfigs.PeakReverseDutyCycle = -1.0;
        leftShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;
        rightShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var leftShooterCurrentConfigs = leftShooterConfigs.CurrentLimits;
        leftShooterCurrentConfigs.StatorCurrentLimitEnable = false;
        leftShooterCurrentConfigs.StatorCurrentLimit = 0;

        var slot0Configs = leftShooterConfigs.Slot0;
        slot0Configs.kP = kP.get();
        slot0Configs.kI = 0.0;
        slot0Configs.kD = kD.get();
        slot0Configs.kS = kS.get();
        slot0Configs.kV = kV.get();

        var leftShooterMotionMagicConfigs = leftShooterConfigs.MotionMagic;
        leftShooterMotionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        leftShooterMotionMagicConfigs.MotionMagicAcceleration = 0.0;
        leftShooterMotionMagicConfigs.MotionMagicJerk = 0.0;
  

        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));

        leftShooterConfigurator.apply(leftShooterConfigs);
        rightShooterConfigurator.apply(rightShooterConfigs);
    }
}
