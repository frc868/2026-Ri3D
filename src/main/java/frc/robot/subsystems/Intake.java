package frc.robot.subsystems;

import java.util.function.Supplier;

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
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.GlobalStates;
import frc.robot.Constants.Intake.IntakePosition;

import static frc.robot.Constants.CAN_BUS_NAME;
import static frc.robot.Constants.Intake.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

@LoggedObject
public class Intake extends SubsystemBase implements BaseSingleJointedArm<IntakePosition>, BaseIntake {
    @Log
    private final TalonFX leftArmMotor;
    @Log
    private final TalonFX rightArmMotor;

    @Log
    private final TalonFX rollerMotor;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> voltageSignal;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut stopRequest = new NeutralOut();

    private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    /** The representation of the "arm" for simulation. */
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED,
            LENGTH_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            IntakePosition.STOW.value.in(Radians));

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    @Log
    private boolean initialized = false;

    @Log
    private double goalPosition;

    @SuppressWarnings("unused")
    private PositionTracker positionTracker;

    public Intake(PositionTracker positionTracker) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = GEARING;

        if (RobotBase.isReal()) {
            motorConfig.CurrentLimits.StatorCurrentLimit = ARM_CURRENT_LIMIT;
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

        leftArmMotor = new TalonFX(LEFT_ARM_MOTOR_ID, CAN_BUS_NAME);
        TalonFXConfigurator leftMotorConfigurator = leftArmMotor.getConfigurator();
        leftMotorConfigurator.apply(motorConfig);

        rightArmMotor = new TalonFX(RIGHT_ARM_MOTOR_ID, CAN_BUS_NAME);
        TalonFXConfigurator rightMotorConfigurator = rightArmMotor.getConfigurator();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfigurator.apply(motorConfig);

        rollerMotor = new TalonFX(ROLLER_MOTOR_ID, CAN_BUS_NAME);

        TalonFXConfigurator rollerMotorConfigurator = rollerMotor.getConfigurator();
        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfig.Feedback.SensorToMechanismRatio = GEARING;

        if (RobotBase.isReal()) {
            rollerMotorConfig.CurrentLimits.StatorCurrentLimit = ROLLER_CURRENT_LIMIT;
            rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        rollerMotorConfigurator.apply(rollerMotorConfig);

        positionSignal = leftArmMotor.getPosition();
        velocitySignal = leftArmMotor.getVelocity();
        accelerationSignal = leftArmMotor.getAcceleration();
        voltageSignal = leftArmMotor.getMotorVoltage();

        leftArmMotor.getClosedLoopReference().setUpdateFrequency(50);
        leftArmMotor.getClosedLoopReferenceSlope().setUpdateFrequency(50);

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
    }

    /**
     * Updated the physics simulation, and sets data on motor controllers based on
     * its results.
     */
    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = leftArmMotor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(0.020);

        talonFXSim.setRawRotorPosition(armSim.getAngleRads() / (2 * Math.PI) * GEARING);
        talonFXSim.setRotorVelocity(armSim.getVelocityRadPerSec() / (2 * Math.PI) * GEARING);
    }

    public boolean getInitialized() {
        return initialized;
    }

    @Log(groups = "components")
    public Pose3d getComponentPose() {
        // since 0 position needs to be horizontal
        return new Pose3d(0.324, 0, 0.193, new Rotation3d(0, -getPosition(), 0));
        // return new Pose3d(1, 1, 1, new Rotation3d(0, getPosition(), 0));
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
        leftArmMotor.setPosition(IntakePosition.STOW.value);
        rightArmMotor.setPosition(IntakePosition.STOW.value);
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
            leftArmMotor.setControl(stopRequest);
            rightArmMotor.setControl(stopRequest);
        } else {
            leftArmMotor.setControl(voltageRequest.withOutput(voltage));
            rightArmMotor.setControl(voltageRequest.withOutput(voltage));
        }
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            if (shouldEnforceSafeties(goalPosition - getPosition())) {
                leftArmMotor.setControl(stopRequest);
                rightArmMotor.setControl(stopRequest);
            } else {
                leftArmMotor.setControl(
                        positionRequest.withPosition(Radians.of(goalPosition).in(Rotations)));
                rightArmMotor.setControl(
                        positionRequest.withPosition(Radians.of(goalPosition).in(Rotations)));
            }
        }).withName("intake.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<IntakePosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get().value.in(Radians)),
                moveToCurrentGoalCommand().until(this::atGoal))
                .withTimeout(1)
                .withName("intake.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get()),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("intake.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> goalPosition + delta.get())
                .withName("intake.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> goalPosition = positionSignal.getValueAsDouble()).andThen(moveToCurrentGoalCommand())
                .withName("intake.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("intake.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("intake.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            leftArmMotor.stopMotor();
            rightArmMotor.stopMotor();
        })
                .andThen(() -> {
                    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
                    leftArmMotor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Coast;
                    leftArmMotor.getConfigurator().apply(motorConfigs);

                    rightArmMotor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Coast;
                    rightArmMotor.getConfigurator().apply(motorConfigs);
                })
                .finallyDo((d) -> {
                    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
                    leftArmMotor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Brake;
                    leftArmMotor.getConfigurator().apply(motorConfigs);

                    rightArmMotor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Brake;
                    rightArmMotor.getConfigurator().apply(motorConfigs);

                    goalPosition = getPosition();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("intake.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("intake.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("intake.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> goalPosition = getPosition());
    }

    @Log
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(getPosition(), goalPosition, TOLERANCE);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(9)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(-9)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("intake.runRollers");
    }

    public Command setRollerVoltageCommand(Supplier<Double> voltage) {
        return Commands.runEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(voltage.get())),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("intake.runRollers");
    }

    public Command jogUpDownCommand() {
        return Commands.sequence(
                moveToPositionCommand(() -> IntakePosition.GROUND).withTimeout(0.5),
                moveToPositionCommand(() -> IntakePosition.JOG).withTimeout(0.5)).repeatedly();
    }

}