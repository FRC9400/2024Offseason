package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX leftShooter = new TalonFX(canIDConstants.leftShooterMotor, "canivore");
    private final TalonFX rightShooter = new TalonFX(canIDConstants.rightShooterMotor, "canivore");
    private TalonFXConfiguration leftShooterConfigs;
    private TalonFXConfiguration rightShooterConfigs;
    private TalonFXConfigurator leftShooterConfigurator;
    private TalonFXConfigurator rightShooterConfigurator;

    private final StatusSignal<Double> leftShooterCurrent = leftShooter.getStatorCurrent();
    private final StatusSignal<Double> rightShooterCurrent = rightShooter.getStatorCurrent();
    private final StatusSignal<Double> leftShooterTemp = leftShooter.getDeviceTemp();
    private final StatusSignal<Double> rightShooterTemp = rightShooter.getDeviceTemp();
    private final StatusSignal<Double> leftShooterSpeedRPS = leftShooter.getRotorVelocity();
    private final StatusSignal<Double> rightShooterSpeedRPS = rightShooter.getRotorVelocity();
    
    private double leftShooterSetpointMPS = 0;
    private double speedRatio = 0;
    private double rightShooterSetpointMPS = 0;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequesetVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    

    LoggedTunableNumber speedRatioTune = new LoggedTunableNumber("Shooter/speedRatio", 1);
    LoggedTunableNumber leftShooterSpeedMPS = new LoggedTunableNumber("Shooter/shooterSpeedMPS", 20);

    LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.085019); //0.068419
    LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0); //0
    LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.2231); //0.16488
    LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.118); //0.11689 0.1121
    
    public ShooterIOTalonFX() {
        this.leftShooterConfigs = new TalonFXConfiguration();
        this.rightShooterConfigs = new TalonFXConfiguration();
        this.leftShooterConfigurator = leftShooter.getConfigurator();
        this.rightShooterConfigurator = rightShooter.getConfigurator();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS);
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
            rightShooterSpeedRPS
        );

        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.currentAmps = new double[] { leftShooterCurrent.getValue(),
                rightShooterCurrent.getValue() };
        inputs.tempFahrenheit = new double[] { leftShooterTemp.getValue(),
                rightShooterTemp.getValue() };
        inputs.shooterSpeedRPS = new double[] { leftShooterSpeedRPS.getValue(),
                rightShooterSpeedRPS.getValue() };
        inputs.shooterSpeedMPS = new double[] {Conversions.RPStoMPS(leftShooterSpeedRPS.getValue(), shooterConstants.wheelCircumferenceMeters, 1), Conversions.RPStoMPS(rightShooterSpeedRPS.getValue(), shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsRPS = new double[] {Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1), Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsMPS = new double[] {leftShooterSetpointMPS, rightShooterSetpointMPS};

    }

    public void updateTunableNumbers() {
        if  (kP.hasChanged(kP.hashCode()) ||
                kD.hasChanged(kD.hashCode()) ||
                kS.hasChanged(kS.hashCode()) ||
                kV.hasChanged(kV.hashCode()) ||
                speedRatioTune.hasChanged(speedRatioTune.hashCode())||
                leftShooterSpeedMPS.hasChanged(leftShooterSpeedMPS.hashCode()) 
                ){
            shooterConfiguration();
        }
    }

    public void setOutput(double volts) {
        leftShooter.setControl(shootRequestVoltage.withOutput(volts));
        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void setVelocity(double velocity, double ratio){
        leftShooterSetpointMPS = velocity;
        rightShooterSetpointMPS = velocity * ratio;
        leftShooter.setControl(leftShootRequesetVelocity.withVelocity(Conversions.MPStoRPS(velocity, shooterConstants.wheelCircumferenceMeters, 1)));
        rightShooter.setControl(rightShootRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity * ratio, shooterConstants.wheelCircumferenceMeters, 1)));
        //leftShooter.setControl(leftShootRequesetVelocity.withVelocity(Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1)));
        //rightShooter.setControl(rightShootRequestVelocity.withVelocity(Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, 1)));
        //leftShooter.setControl(shootRequestMotionMagic.withVelocity(velocityRPS));
    }

    public void zeroVelocity(){
        leftShooterSetpointMPS = 0;
        rightShooterSetpointMPS = 0;
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

        rightShooterMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightShooterMotorConfigs.PeakForwardDutyCycle = 1.0;
        rightShooterMotorConfigs.PeakReverseDutyCycle = -1.0;
        rightShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var leftShooterCurrentConfigs = leftShooterConfigs.CurrentLimits;
        leftShooterCurrentConfigs.StatorCurrentLimitEnable = true;
        leftShooterCurrentConfigs.StatorCurrentLimit = 50;

        var rightShooterCurrentConfigs = rightShooterConfigs.CurrentLimits;
        rightShooterCurrentConfigs.StatorCurrentLimitEnable = true;
        leftShooterCurrentConfigs.StatorCurrentLimit = 50;

        var leftSlot0Configs = leftShooterConfigs.Slot0;
        leftSlot0Configs.kP = kP.get();
        leftSlot0Configs.kI = 0.0;
        leftSlot0Configs.kD = kD.get();
        leftSlot0Configs.kS = kS.get();
        leftSlot0Configs.kV = kV.get();
        leftSlot0Configs.kA = 0.0085063;

        var rightSlot0Configs = rightShooterConfigs.Slot0;
        rightSlot0Configs.kP = kP.get();
        rightSlot0Configs.kI = 0.0;
        rightSlot0Configs.kD = kD.get();
        rightSlot0Configs.kS = kS.get();
        rightSlot0Configs.kV = kV.get();
        rightSlot0Configs.kA = 0.0085063;

        leftShooterConfigurator.apply(leftShooterConfigs);
        rightShooterConfigurator.apply(rightShooterConfigs);
        /* 
        leftShooterSetpointMPS = leftShooterSpeedMPS.get();
        speedRatio = speedRatioTune.get();
        rightShooterSetpointMPS = leftShooterSetpointMPS * speedRatio;
        */
    }
}
