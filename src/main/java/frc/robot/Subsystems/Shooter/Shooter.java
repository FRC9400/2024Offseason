package frc.robot.Subsystems.Shooter;

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


public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine shooterRoutine;
    private final SysIdRoutine armRoutine;

    private ShooterStates state = ShooterStates.IDLE;
    private double shooterVolts = 0;
    private double[] shooterVelocity = {0, 0}; //velocity, ratio
    private double armSetpointDeg = 0;
    private double armVolts = 0;

    public enum ShooterStates{
        IDLE,
        SHOOT,
        VOLTAGE
    }

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        shooterRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(6),null, 
                    (state) -> SignalLogger.writeString("state", state.toString())), 
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> shooterIO.requestShooterVoltage(volts.in(Volts)), null, 
                    this));
        armRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(5), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> shooterIO.requestArmVoltage(volts.in(Volts)), null,
                    this));
    }

    public Command shooterSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            shooterRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.shooterSpeedMPS[0] > 30),
                this.runOnce(() -> shooterIO.requestShooterVoltage(0)),
                Commands.waitSeconds(1),
            shooterRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.shooterSpeedMPS[0] < -30),
                this.runOnce(() -> shooterIO.requestShooterVoltage(0)),
                Commands.waitSeconds(1),  

            shooterRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.shooterSpeedMPS[0] > 15),
                this.runOnce(() -> shooterIO.requestShooterVoltage(0)),
                Commands.waitSeconds(1),  

            shooterRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.shooterSpeedMPS[0] < -15),
                this.runOnce(() -> shooterIO.requestShooterVoltage(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }

    public Command armSysIdCmd(){
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                armRoutine
                        .quasistatic(Direction.kForward)
                        .until(() -> Math.abs(inputs.armPosDeg[0]) > 140),
                this.runOnce(() -> shooterIO.requestArmVoltage(0)),
                Commands.waitSeconds(1),
                armRoutine
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.armPosDeg[0] < 5),
                this.runOnce(() -> shooterIO.requestArmVoltage(0)),
                Commands.waitSeconds(1),

                armRoutine
                        .dynamic(Direction.kForward)
                        .until(() -> Math.abs(inputs.armPosDeg[0]) > 140),
                this.runOnce(() -> shooterIO.requestArmVoltage(0)),
                Commands.waitSeconds(1),

                armRoutine
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.armPosDeg[0] < 5),
                this.runOnce(() -> shooterIO.requestArmVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    } 

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        //Logger.recordOutput("ShooterState", state);

        switch(state){
            case IDLE:
                shooterIO.requestArmVoltage(0);
                shooterIO.requestShooterVoltage(0);
                break;
            case SHOOT:
                shooterIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterIO.requestSetpoint(armSetpointDeg);
                break;
            case VOLTAGE:
                shooterIO.requestArmVoltage(armVolts);
                shooterIO.requestShooterVoltage(shooterVolts);
                break;
        }
    }

    public void requestVoltage(double shooterVoltage, double armVoltage){
        this.shooterVolts = shooterVoltage;
        this.armVolts = armVoltage;
        setState(ShooterStates.VOLTAGE);
    }

    public void requestShoot(double velocity, double ratio, double armSetpointDeg){
        this.shooterVelocity[0] = velocity;
        this.shooterVelocity[1] = ratio;
        this.armSetpointDeg = armSetpointDeg;
        setState(ShooterStates.SHOOT);
    }

    public void setState(ShooterStates nextState){
        this.state = nextState;
    }

    public ShooterStates getState(){
        return this.state;
    }

}
