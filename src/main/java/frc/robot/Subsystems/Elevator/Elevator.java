package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants.elevatorConstants;

public class Elevator {
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorState state = ElevatorState.IDLE;
    private double elevatorSetpoints = 0; 
    private double jogInput = 0.0;
    private double stateStartTime = 0;

    public enum ElevatorState{
        IDLE,
        HOMING, 
        JOG,
        SETPOINT,
        CLIMB,
        ZEROSENSOR
    }

    public void Loop() {
        elevatorIO.updateInputs(inputs);
        elevatorIO.updateTunableNumbers();
        Logger.recordOutput("ElevatorState", state);
        Logger.processInputs("Elevator", inputs);

        switch(state){
            case IDLE:
                elevatorIO.setOutput(0);
                break;
            case HOMING:
                elevatorIO.setOutput(-3);
                break;
            case JOG:
                elevatorIO.setOutput(jogInput);
                break;
            case SETPOINT:
                elevatorIO.goToSetpoint(elevatorSetpoints);
                break;
            case ZEROSENSOR:
                elevatorIO.zeroSensor();
            default:
                break;
        }
    }
    public void requestElevatorHeight(double height, boolean climb){
        elevatorSetpoints = height;
        elevatorIO.setMotionMagicConfigs(climb);
        setState(ElevatorState.SETPOINT);
    }

    public void requestJog(double jogInput){
        this.jogInput = jogInput;
        setState(ElevatorState.JOG);
    }

    public void setState(ElevatorState nextState){
        this.state = nextState;
        stateStartTime = RobotController.getFPGATime()/1E6;
    }

    public boolean atElevatorSetpoint(double height){
        return Math.abs(inputs.elevatorHeightMeters - height) < Units.inchesToMeters(1);
    }

    public double getElevatorVelMPS(){
        return inputs.elevatorVelMPS;
    }
     
    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        
    }

    public void elevatorConfiguration() {
        elevatorIO.elevatorConfiguration();
    }

 

    public ElevatorState getState(){
        return this.state;
    }
    
}
