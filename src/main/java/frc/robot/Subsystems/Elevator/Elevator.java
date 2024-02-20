package frc.robot.Subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private final SysIdRoutine elevatorRoutine;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


    @Override
    public void periodic(){
  
        elevatorIO.updateInputs(inputs);
        elevatorIO.updateTunableNumbers(); 
        Logger.processInputs("Elevator", inputs);
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(2),null, (state) -> SignalLogger.writeString("state", state.toString())), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> elevatorIO.setOutput(volts.in(Volts)), null, this));
      }

      public void setHeight(double setPoint){
        elevatorIO.setHeight(setPoint);
    }

    public Command runSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            elevatorRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),
            elevatorRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.elevatorHeightMeters > 0.2),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),  

            elevatorRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.elevatorHeightMeters > 0.2),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1),  

            elevatorRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.elevatorHeightMeters > 0.2),
                this.runOnce(() -> elevatorIO.setOutput(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }


    public void testOutput(){
        elevatorIO.testOutput();
    }

    public void setOutput(double output){
        elevatorIO.setOutput(output);
    }
    
    public void setElevator(double x){
        elevatorIO.setElevator(x);
    }

    public void homing(){
        elevatorIO.homing();
    }
 
    public void elevatorConfiguration(){
        elevatorIO.elevatorConfiguration();
    }
    
}

