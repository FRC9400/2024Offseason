package frc.robot.Subsystems.OTB_Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.commons.Conversions;
import frc.robot.Constants.otbIntakeConstants;

public class OTB_IntakeIOTalonFX implements OTB_IntakeIO {
    private final TalonFX pivot = new TalonFX(0);
    private final TalonFX intake = new TalonFX(0);
    private final TalonFX indexer = new TalonFX(0);

    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfiguration intakeConfigs;
    private final TalonFXConfiguration indexerConfigs;

    MotionMagicVoltage pivotMotionMagicRequest;
    VoltageOut pivotVoltageRequest;
    VoltageOut intakeVoltageRequest;
    VoltageOut indexerVoltageRequest;

    double pivotSetpoint;
    double indexerSetpointVolts;
    double intakeSetpointVolts;

    private final StatusSignal<Double> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Double> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<Double> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Double> pivotPos = pivot.getRotorPosition();

    private final StatusSignal<Double> intakeCurrent = intake.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intake.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intake.getRotorVelocity();

    private final StatusSignal<Double> indexerCurrent = indexer.getStatorCurrent();
    private final StatusSignal<Double> indexerTemp = indexer.getDeviceTemp();
    private final StatusSignal<Double> indexerRPS = indexer.getRotorVelocity();

    
    public OTB_IntakeIOTalonFX() {

        pivotConfigs = new TalonFXConfiguration();
        intakeConfigs = new TalonFXConfiguration();
        indexerConfigs = new TalonFXConfiguration();

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
        slot0Configs.kA = 0.0;
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

        var indexerMotorOutputConfigs = indexerConfigs.MotorOutput;
        indexerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        indexerMotorOutputConfigs.Inverted = otbIntakeConstants.indexerInvert;

        var indexerCurrentLimitConfigs = indexerConfigs.CurrentLimits;
        indexerCurrentLimitConfigs.StatorCurrentLimit = otbIntakeConstants.indexerCurrentLimit;
        indexerCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        indexerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        pivot.getConfigurator().apply(pivotConfigs);
        intake.getConfigurator().apply(intakeConfigs);
        indexer.getConfigurator().apply(indexerConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp,
                intakeTemp,
                intakeCurrent,
                intakeRPS,
                indexerTemp,
                indexerCurrent,
                indexerRPS
               );

        intake.optimizeBusUtilization();
    }

    public void updateInputs(OTB_IntakeIOInputs otbIntakeInputs){
        BaseStatusSignal.refreshAll(
          pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            intakeTemp,
            intakeCurrent,
            intakeRPS,
            indexerTemp,
            indexerCurrent,
            indexerRPS
        );
        otbIntakeInputs.pivotAppliedVolts = pivotVoltageRequest.Output;
        otbIntakeInputs.pivotCurrent = pivotCurrent.getValue();
        otbIntakeInputs.pivotPosDeg = Conversions.RotationsToDegrees(pivotPos.getValue(), otbIntakeConstants.gearRatio);
        otbIntakeInputs.pivotPosRot = pivotPos.getValue();
        otbIntakeInputs.pivotSetpointDeg = pivotSetpoint;
        otbIntakeInputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpoint, otbIntakeConstants.gearRatio);
        otbIntakeInputs.pivotTemperature = pivotTemp.getValue();
        otbIntakeInputs.pivotRPS = pivotRPS.getValue();

        otbIntakeInputs.intakeTemperature = intakeTemp.getValue();
        otbIntakeInputs.intakeAppliedVolts = intakeVoltageRequest.Output;
        otbIntakeInputs.intakeCurrent = intakeCurrent.getValue();
        otbIntakeInputs.intakeRPS = intakeRPS.getValue();

        otbIntakeInputs.indexerTemperature = indexerTemp.getValue();
        otbIntakeInputs.indexerAppliedVolts = indexerVoltageRequest.Output;
        otbIntakeInputs.indexerCurrent = indexerCurrent.getValue();
        otbIntakeInputs.indexerRPS = indexerRPS.getValue();
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
        intakeSetpointVolts = voltage;
        intake.setControl(intakeVoltageRequest.withOutput(intakeSetpointVolts));
    }

    public void requestIndexerVoltage(double voltage) {
        indexerSetpointVolts = voltage;
        indexer.setControl(indexerVoltageRequest.withOutput(indexerSetpointVolts));
    }

    public void zeroPosition(){
        pivot.setPosition(0);
    }
}