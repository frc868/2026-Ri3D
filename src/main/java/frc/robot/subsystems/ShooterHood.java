package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.techhounds.houndutil.houndlib.EqualsUtil;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BasePivot;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.Tunable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterHood.HoodPosition;
import frc.robot.GlobalStates;

import static frc.robot.Constants.ShooterHood.*;
import static frc.robot.Constants.CAN_BUS_NAME;

import java.util.function.Supplier;

@LoggedObject
public class ShooterHood extends SubsystemBase implements BasePivot<HoodPosition> {
    @Log
    private final TalonFX motor;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> voltageSignal;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut stopRequest = new NeutralOut();

    /** The representation of the "arm" for simulation. */
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED,
            LENGTH_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            MAX_ANGLE_RADIANS);

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    @Log
    private boolean initialized = false;

    @Log
    private double goalPosition;

    @Log
    @Tunable
    private double targetAngle;

    public ShooterHood(PositionTracker positionTracker) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = GEARING;

        if (RobotBase.isReal()) {
            motorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
            motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kG = kG;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kA = kA;
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY_ROTATIONS_PER_SECOND;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;

        motor = new TalonFX(MOTOR_ID, CAN_BUS_NAME);
        TalonFXConfigurator leftMotorConfigurator = motor.getConfigurator();
        leftMotorConfigurator.apply(motorConfig);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();
        voltageSignal = motor.getMotorVoltage();

        motor.getClosedLoopReference().setUpdateFrequency(50);
        motor.getClosedLoopReferenceSlope().setUpdateFrequency(50);

        SignalManager.register(
                CAN_BUS_NAME,
                positionSignal, velocitySignal, accelerationSignal, voltageSignal);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            setVoltage(volts.in(Volts));
                        },
                        log -> {
                            log.motor("left")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(voltageSignal.getValueAsDouble(),
                                            Volts))
                                    .angularPosition(
                                            sysidPositionMeasure.mut_replace(positionSignal.getValueAsDouble(),
                                                    Rotations))
                                    .angularVelocity(sysidVelocityMeasure.mut_replace(velocitySignal.getValueAsDouble(),
                                            RotationsPerSecond));
                        },
                        this));

        setDefaultCommand(moveToCurrentGoalCommand());
        // setDefaultCommand(Commands.parallel(
        // Commands.run(() -> goalPosition =
        // SHOT_ANGLE_TO_HOOD_ANGLE.get(Units.degreesToRadians(targetAngle))),
        // moveToCurrentGoalCommand()));
    }

    /**
     * Updated the physics simulation, and sets data on motor controllers based on
     * its results.
     */
    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = motor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        armSim.setInputVoltage(-motorVoltage.in(Volts));
        armSim.update(0.020);

        // negative because inversion state
        talonFXSim.setRawRotorPosition(-armSim.getAngleRads() / (2 * Math.PI) * GEARING);
        talonFXSim.setRotorVelocity(-armSim.getVelocityRadPerSec() / (2 * Math.PI) * GEARING);
    }

    public boolean getInitialized() {
        return initialized;
    }

    @Log(groups = "components")
    public Pose3d getComponentPose() {
        return new Pose3d(-0.146, 0, 0.444, new Rotation3d(0, -getPosition(), 0));
    }

    @Log
    @Override
    public double getPosition() {
        return positionSignal.getValue().in(Radians);
    }

    @Log
    public double getVelocity() {
        return velocitySignal.getValue().in(RadiansPerSecond);
    }

    @Override
    public void resetPosition() {
        motor.setPosition(Units.radiansToRotations(HoodPosition.BOTTOM.value));
        initialized = true;
    }

    public boolean shouldEnforceSafeties(double intendedDirection) {
        if (Utils.applySoftStops(intendedDirection, getPosition(), MIN_ANGLE_RADIANS,
                MAX_ANGLE_RADIANS) == 0.0)
            return true;

        if (!GlobalStates.INITIALIZED.enabled()) {
            return true;
        }

        return false;
    }

    @Override
    public void setVoltage(double voltage) {
        if (shouldEnforceSafeties(voltage)) {
            motor.setControl(stopRequest);
        } else {
            motor.setControl(voltageRequest.withOutput(voltage));
        }
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            if (shouldEnforceSafeties(goalPosition - getPosition())) {
                motor.setControl(stopRequest);
            } else {
                motor.setControl(
                        positionRequest.withPosition(Radians.of(goalPosition).in(Rotations)));
            }
        }).withName("shooterHood.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<HoodPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get().value),
                moveToCurrentGoalCommand().until(this::atGoal))
                .withTimeout(1)
                .withName("shooterHood.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.parallel(
                Commands.run(() -> goalPosition = goalPositionSupplier.get()),
                moveToCurrentGoalCommand()).withName("shooterHood.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> goalPosition + delta.get())
                .withName("shooterHood.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> goalPosition = positionSignal.getValueAsDouble()).andThen(moveToCurrentGoalCommand())
                .withName("shooterHood.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("shooterHood.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("shooterHood.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            motor.stopMotor();
        })
                .andThen(() -> {
                    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
                    motor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Coast;
                    motor.getConfigurator().apply(motorConfigs);
                })
                .finallyDo((d) -> {
                    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
                    motor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Brake;
                    motor.getConfigurator().apply(motorConfigs);

                    goalPosition = getPosition();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooterHood.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooterHood.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooterHood.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> goalPosition = getPosition());
    }

    @Log
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(getPosition(), goalPosition, TOLERANCE);
    }
}
