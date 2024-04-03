package frc.robot.Subsystems.OTB_Intake;

import java.lang.Math;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.otbIntakeConstants;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class OTB_IntakeIOTalonFX implements OTB_IntakeIO {
    private final TalonFX pivot = new TalonFX(canIDConstants.otbIntakePivotMotor, "canivore");
    private final TalonFX intake = new TalonFX(canIDConstants.otbIntakeMotor, "canivore");
    private final TalonFXConfigurator pivotConfigurator;
    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfigurator intakeConfigurator;
    private final TalonFXConfiguration intakeConfigs;
    MotionMagicVoltage pivotMotionMagicRequest;
    VoltageOut pivotVoltageRequest;
    VoltageOut intakeVoltageRequest;
    double pivotSetpoint;

    private final StatusSignal<Double> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Double> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<Double> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Double> pivotPos = pivot.getRotorPosition();

    private final StatusSignal<Double> intakeCurrent = intake.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intake.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intake.getRotorVelocity();

    public OTB_IntakeIOTalonFX() {
        pivotConfigurator = pivot.getConfigurator();
        pivotConfigs = new TalonFXConfiguration();

        intakeConfigurator = intake.getConfigurator();
        intakeConfigs = new TalonFXConfiguration();

        var pivotMotorOuputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOuputConfigs.Inverted = otbIntakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = otbIntakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kP = 0.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kS = 0.0;
        slot0Configs.kV = 0.0;
        slot0Configs.kG = 0.0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        motionMagicConfigs.MotionMagicAcceleration = 0.0;
        motionMagicConfigs.MotionMagicJerk = 0.0;

        var feedbackConfigs = pivotConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivot.setPosition(0);

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = otbIntakeConstants.intakeInvert;

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = otbIntakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        pivotConfigurator.apply(pivotConfigs);
        intakeConfigurator.apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp,
                intakeCurrent,
                intakeTemp,
                intakeRPS);

        intake.optimizeBusUtilization();
    }

    public void updateInputs(OTB_IntakeIOInputs otbIntakeInputs){
        BaseStatusSignal.refreshAll(
          pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            intakeCurrent,
            intakeTemp,
            intakeRPS  
        );
        otbIntakeInputs.pivotAppliedVolts = pivotVoltageRequest.Output;
        otbIntakeInputs.pivotCurrent = pivotCurrent.getValue();
        otbIntakeInputs.pivotPosDeg = Conversions.DegreesToRotations(pivotPos.getValue(), otbIntakeConstants.gearRatio);
        otbIntakeInputs.pivotPosRot = pivotPos.getValue();
        otbIntakeInputs.pivotSetpointDeg = pivotSetpoint;
        otbIntakeInputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpoint, otbIntakeConstants.gearRatio);
        otbIntakeInputs.pivotTemperature = pivotTemp.getValue();
        otbIntakeInputs.pivotRPS = pivotRPS.getValue();

        otbIntakeInputs.intakeTemperature = intakeTemp.getValue();
        otbIntakeInputs.intakeAppliedVolts = intakeVoltageRequest.Output;
        otbIntakeInputs.intakeCurrent = intakeCurrent.getValue();
        otbIntakeInputs.intakeRPS = intakeRPS.getValue();
    }

    public void requestPivotVoltage(double voltage) {
        pivot.setControl(pivotVoltageRequest.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees) {
        pivotSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, otbIntakeConstants.gearRatio);
        pivot.setControl(pivotMotionMagicRequest.withPosition(pivotSetpointRotations));
    }

    public void requestIntakeVoltage(double voltage) {
        intake.setControl(intakeVoltageRequest.withOutput(voltage));
    }

}