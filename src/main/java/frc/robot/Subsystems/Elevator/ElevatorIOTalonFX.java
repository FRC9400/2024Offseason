package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;


import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;
import frc.commons.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO{
    private final TalonFX leftMotor = new TalonFX(canIDConstants.leftElevatorMotor, "canivore");
    private final TalonFX rightMotor = new TalonFX(canIDConstants.rightElevatorMotor, "canivore");
    private final TalonFXConfiguration leftMotorConfigs;
    private final TalonFXConfiguration rightMotorConfigs;
    private final TalonFXConfigurator leftMotorConfigurator;
    private final TalonFXConfigurator rightMotorConfigurator;

    private final StatusSignal<Double> leftElevatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Double> rightElevatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Double> leftElevatorTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Double> rightElevatorTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<Double> leftElevatorSpeedRPS = leftMotor.getRotorVelocity();
    private final StatusSignal<Double> leftElevatorPos = leftMotor.getPosition();

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true); 
    private VoltageOut voltageOutRequest =  new VoltageOut(0).withEnableFOC(true);
    
    double setPointMeters;
    

    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 8.413);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0030141);
    LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.058684);
    LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0044);
    LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG",0.0662029); 
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber( "Elevator/kMotionCruiseVelocity",10); //48
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber( "Elevator/kMotionAcceleration",20); //96
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Elevator/kMotionJerk",10000);
    


    public ElevatorIOTalonFX(){
        leftMotorConfigurator = leftMotor.getConfigurator();
        rightMotorConfigurator = rightMotor.getConfigurator();
        leftMotorConfigs = new TalonFXConfiguration();
        rightMotorConfigs = new TalonFXConfiguration();
        setPointMeters = 0;  

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorSpeedRPS,
            leftElevatorPos);
            leftMotor.optimizeBusUtilization();
            rightMotor.optimizeBusUtilization();
    }
    
    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(
           leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorSpeedRPS,
            leftElevatorPos 
        );

        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.setPointMeters = setPointMeters;
        inputs.elevatorVelMPS = Conversions.RPStoMPS(leftElevatorSpeedRPS.getValue(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.elevatorHeightMeters = Conversions.RotationsToMeters(leftElevatorPos.getValue(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.currentAmps = new double[] {leftElevatorCurrent.getValue(), rightElevatorCurrent.getValue()};
        inputs.tempFahrenheit = new double[] {leftElevatorTemp.getValue(), rightElevatorTemp.getValue()};
        
    }

    public void updateTunableNumbers() {
        if (
          kD.hasChanged(kD.hashCode()) ||
          kG.hasChanged(kG.hashCode()) ||
          kS.hasChanged(kS.hashCode()) ||
          kP.hasChanged(kP.hashCode()) ||
          kV.hasChanged(kV.hashCode())||
          kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode()) ||
          kMotionCruiseVelocity.hasChanged(kMotionCruiseVelocity.hashCode())
        ) {
          elevatorConfiguration();
        }
      }

    public void zeroSensor(){
        leftMotor.setPosition(0);
    }

    public void setOutput(double output){
        leftMotor.setControl(voltageOutRequest.withOutput(output));
    }

    public void driveElevator(double setPointMeters, boolean climb){
        double setPointRotations = Conversions.metersToRotations(setPointMeters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        leftMotor.setControl(motionMagicRequest.withPosition(setPointRotations));     
    }

     
    public void elevatorConfiguration(){
        var leftMotorOuputConfigs = leftMotorConfigs.MotorOutput;
        var rightMotorOutputConfigs = rightMotorConfigs.MotorOutput;
        leftMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorOuputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
        var slot0Configs = leftMotorConfigs.Slot0;
        slot0Configs.kP = kP.get();
        slot0Configs.kI = 0.0;
        slot0Configs.kD = kD.get();
        slot0Configs.kS = kS.get();
        slot0Configs.kV = kV.get();
        slot0Configs.kG = kG.get();
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        
        var motionMagicConfigs = leftMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kMotionCruiseVelocity.get();
        motionMagicConfigs.MotionMagicAcceleration = kMotionAcceleration.get();
        motionMagicConfigs.MotionMagicJerk = kMotionJerk.get();

        var feedbackConfigs = leftMotorConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.setPosition(0);
        
        leftMotorConfigurator.apply(leftMotorConfigs);
        rightMotorConfigurator.apply(rightMotorConfigs);
        
    }    

}


    