package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;


import frc.commons.Conversions;
import frc.robot.Constants.elevatorConstants;
import frc.commons.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFXConfiguration leftMotorConfigs;
    private final TalonFXConfiguration rightMotorConfigs;
    private final TalonFXConfigurator leftMotorConfigurator;
    private final TalonFXConfigurator rightMotorConfigurator;

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true); 
    private MotionMagicExpoVoltage motionMagicExpoRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
    private VoltageOut voltageOutRequest =  new VoltageOut(0).withEnableFOC(true);

    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
    LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber( "Elevator/kMotionCruiseVelocity",0.0);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber( "Elevator/kMotionAcceleration",0.0);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Elevator/kMotionJerk",0.0);
    LoggedTunableNumber kMotionExpokV = new LoggedTunableNumber("Elevator/kMotionExpokV",0.0);
    LoggedTunableNumber kMotionExpokA = new LoggedTunableNumber("Elevator/kMotionExpokA",0.0);
    LoggedTunableNumber elevatorVolts = new LoggedTunableNumber("Elevator/ElevatorVolts", 2);



    public ElevatorIOTalonFX(int leftMotorID, int rightMotorID){
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);
        leftMotorConfigurator = leftMotor.getConfigurator();
        rightMotorConfigurator = rightMotor.getConfigurator();
        leftMotorConfigs = new TalonFXConfiguration();
        rightMotorConfigs = new TalonFXConfiguration(); 

    }
    
    public void updateInputs(ElevatorIOInputs elevatorInputs){
        elevatorInputs.appliedVolts = voltageOutRequest.Output;
        elevatorInputs.elevatorHeightMeters = Conversions.RotationsToMeters(leftMotor.getRotorPosition().getValue(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        elevatorInputs.currentAmps = new double[] {leftMotor.getStatorCurrent().getValue(), rightMotor.getStatorCurrent().getValue()};
        elevatorInputs.tempFahrenheit = new double[] {leftMotor.getDeviceTemp().getValue(), rightMotor.getDeviceTemp().getValue()};
        elevatorInputs.rotorVelocityRPS = new double[] {leftMotor.getRotorVelocity().getValue(), rightMotor.getRotorVelocity().getValue()};
        
    }

    public void updateTunableNumbers() {
        if (
          kD.hasChanged(kD.hashCode()) ||
          kG.hasChanged(kG.hashCode()) ||
          kS.hasChanged(kS.hashCode()) ||
          kP.hasChanged(kP.hashCode()) ||
          kV.hasChanged(kV.hashCode())||
          kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode()) ||
          kMotionCruiseVelocity.hasChanged(kMotionCruiseVelocity.hashCode()) ||
          kMotionExpokV.hasChanged(kMotionExpokV.hashCode()) ||
          kMotionExpokA.hasChanged(kMotionExpokA.hashCode()) ||
          elevatorVolts.hasChanged(elevatorVolts.hashCode())
        ) {
          elevatorConfiguration();
        }
      }



    public void testOutput(){
        leftMotor.setControl(voltageOutRequest.withOutput(elevatorVolts.get()));
    }

    public void setVoltage(double output){
        leftMotor.setControl(voltageOutRequest.withOutput(output));
    }

    public void setElevator(double setPointMeters){
        leftMotor.setControl(motionMagicRequest.withPosition(setPointMeters)); 
        //leftMotor.setControl(motionMagicExpoRequest.withPosition(setPointMeters));
    }

    public void resetHeight(){
        leftMotor.setPosition(0);
    }

    public void elevatorConfiguration(){
        var leftMotorOuputConfigs = leftMotorConfigs.MotorOutput;
        var rightMotorOutputConfigs = rightMotorConfigs.MotorOutput;
        leftMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorOuputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        //rightMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        
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
        motionMagicConfigs.MotionMagicExpo_kV = kMotionExpokV.get();
        motionMagicConfigs.MotionMagicExpo_kA = kMotionExpokA.get();

        var feedbackConfigs = leftMotorConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.setPosition(0);
        
        leftMotorConfigurator.apply(leftMotorConfigs);
        rightMotorConfigurator.apply(rightMotorConfigs);
        
    }    

}


    