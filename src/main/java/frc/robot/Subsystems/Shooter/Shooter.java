package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.shooterConstants;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Volts;


public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine shooterRoutine;
    private final SysIdRoutine armRoutine;

    private ShooterStates state = ShooterStates.IDLE;

    private double volts = 0;
    private double velocity = 0;
    private double setpointDeg = 0;

    public enum ShooterStates{
        IDLE,
        SHOOT,
        PREPARE,
        AMP
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

    public Command armSysIdCmd() {
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

    public void Loop(){
        shooterIO.updateInputs(inputs);

        switch(state){
            case IDLE:
                shooterIO.requestShooterVoltage(0);
                break;
            case PREPARE:
                shooterIO.requestVelocity(velocity, shooterConstants.shooterGearRatio);
                shooterIO.requestSetpoint(setpointDeg);
                break;
            case SHOOT:
                shooterIO.requestVelocity(velocity, shooterConstants.shooterGearRatio);
                shooterIO.requestSetpoint(setpointDeg);
                break;
            case AMP:
                shooterIO.requestAmpRollerVoltage(volts);
                shooterIO.requestSetpoint(setpointDeg);
                break;
        }
    }
  
    public void prepShoot(double velocity, double deg){
        this.velocity=velocity;
        this.setpointDeg = deg;
        setState(ShooterStates.PREPARE);
    }

    public void shoot(double velocity, double deg){
        this.velocity = velocity;
        this.setpointDeg = deg;
        setState(ShooterStates.SHOOT);
    }

    public void amp(double volts, double deg){
        this.volts = volts;
        this.setpointDeg = deg;
        setState(ShooterStates.AMP);
    }

    public void setState(ShooterStates nextState){
        this.state=nextState;
    }

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        }
}
