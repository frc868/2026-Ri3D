package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.techhounds.houndutil.houndlib.EqualsUtil;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.Tunable;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CAN_BUS_NAME;
import static frc.robot.Constants.Shooter.*;

import java.util.function.Supplier;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@LoggedObject
public class Shooter extends SubsystemBase implements BaseShooter {
    @Log
    private final TalonFX primaryMotor;
    @Log
    private final TalonFX secondaryMotor;
    // @Log
    // private final TalonFX tertiaryMotor;
    // @Log
    // private final TalonFX quaternaryMotor;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(MOTOR_GEARBOX_REPR, MOMENT_OF_INERTIA_KG_METERS_SQUARED, GEARING),
            MOTOR_GEARBOX_REPR);

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    @Log
    private double goalVelocity;

    @Log
    @Tunable
    public double targetVelocity = 10;

    public Shooter() {
        TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.Feedback.SensorToMechanismRatio = GEARING;

        if (RobotBase.isReal()) {
            shooterMotorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
            shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        shooterMotorConfig.Slot0.kS = kS;
        shooterMotorConfig.Slot0.kV = kV;
        shooterMotorConfig.Slot0.kA = kA;
        shooterMotorConfig.Slot0.kP = kP;
        shooterMotorConfig.Slot0.kI = kI;
        shooterMotorConfig.Slot0.kD = kD;
        shooterMotorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        primaryMotor = new TalonFX(PRIMARY_MOTOR_ID, CAN_BUS_NAME);
        primaryMotor.getConfigurator().apply(shooterMotorConfig);

        secondaryMotor = new TalonFX(PRIMARY_MOTOR_ID, CAN_BUS_NAME);
        secondaryMotor.getConfigurator().apply(shooterMotorConfig);

        // tertiaryMotor = new TalonFX(PRIMARY_MOTOR_ID, CAN_BUS_NAME);
        // tertiaryMotor.getConfigurator().apply(shooterMotorConfig);

        // quaternaryMotor = new TalonFX(PRIMARY_MOTOR_ID, CAN_BUS_NAME);
        // quaternaryMotor.getConfigurator().apply(shooterMotorConfig);

        secondaryMotor.setControl(new Follower(primaryMotor.getDeviceID(), true));
        // tertiaryMotor.setControl(new Follower(primaryMotor.getDeviceID(), false));
        // quaternaryMotor.setControl(new Follower(primaryMotor.getDeviceID(), false));

        positionSignal = primaryMotor.getPosition();
        velocitySignal = primaryMotor.getVelocity();
        voltageSignal = primaryMotor.getMotorVoltage();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.in(Volts)),
                        log -> {
                            log.motor("left")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(voltageSignal.getValueAsDouble(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure
                                            .mut_replace(positionSignal.getValueAsDouble(), Rotations))
                                    .angularVelocity(
                                            sysidVelocityMeasure.mut_replace(velocitySignal.getValueAsDouble(),
                                                    RotationsPerSecond));
                        },
                        this));

        // setDefaultCommand(spinAtVelocityCommand(() -> targetVelocity));
        setDefaultCommand(spinAtVelocityCommand(() -> 0.0));
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = primaryMotor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();
        // set the input (the voltage of the motor)
        flywheelSim.setInputVoltage(motorVoltage.in(Volts));
        flywheelSim.update(0.020);

        talonFXSim.setRotorVelocity(flywheelSim.getAngularVelocity().times(GEARING));
    }

    @Override
    public double getVelocity() {
        return velocitySignal.getValue().in(RotationsPerSecond);
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    // rot/sec
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run(() -> {
            goalVelocity = goalVelocitySupplier.get();
            primaryMotor.setControl(velocityRequest.withVelocity(goalVelocitySupplier.get()));
        }).withName("shooter.spinAtVelocity");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("shooter.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return Commands.none();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
    }

    @Log
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(getVelocity(), goalVelocity, TOLERANCE);
    }
}
