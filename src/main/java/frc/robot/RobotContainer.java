// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.BallSimulator;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

import static frc.robot.FieldConstants.*;

public class RobotContainer {
    PositionTracker positionTracker = new PositionTracker();
    @Log
    BallSimulator ballSimulator = new BallSimulator(BALL_CONSTANTS, FIELD_LENGTH, FIELD_WIDTH);

    @Log(groups = "subsystems")
    Drivetrain drivetrain = new Drivetrain(positionTracker);
    @Log(groups = "subsystems")
    Shooter shooter = new Shooter();
    @Log(groups = "subsystems")
    ShooterHood shooterHood = new ShooterHood(positionTracker);
    @Log(groups = "subsystems")
    Hopper hopper = new Hopper();
    @Log(groups = "subsystems")
    Intake intake = new Intake(positionTracker);

    @Log(groups = "subsystems")
    Vision vision = new Vision();
    @Log(groups = "subsystems")
    LEDs leds = new LEDs();

    @Log
    ShotCalculator shotCalculator = new ShotCalculator(drivetrain);

    @Log
    Superstructure superstructure = new Superstructure(drivetrain, shooter, shooterHood, hopper, intake, leds,
            ballSimulator,
            shotCalculator);

    @Log(groups = "subsystems")
    HoundBrian houndbrian = new HoundBrian(drivetrain, shooterHood, intake, leds);

    @Log
    private final Supplier<Boolean> initialized = GlobalStates.INITIALIZED::enabled;

    @SendableLog
    CommandScheduler scheduler = CommandScheduler.getInstance();

    @Log(groups = "timing")
    private Supplier<Double> odometryLoopTimeMs = () -> drivetrain.getOdometryLoopTime();

    @Log
    private final Supplier<Double> matchTimer = DriverStation::getMatchTime;

    @Log
    private final Pose3d target = FieldConstants.Hub.CENTER;

    @Log
    private final Pose3d zero = Pose3d.kZero;
    @Log
    private final Pose3d componentPose = new Pose3d(1, 1, 1, Rotation3d.kZero);

    @Log
    private final Supplier<Double> distance = () -> drivetrain.getPose().getTranslation()
            .getDistance(target.getTranslation().toTranslation2d());

    public RobotContainer() {
        configureBindings();
        configureAuto();
        configureTriggers();

        LoggingManager.getInstance().registerObject(this);
        LoggingManager.getInstance().registerMetadata(Constants.BUILD_METADATA);
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();
        SignalLogger.enableAutoLogging(false);

        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        vision.setSimPoseSupplier(drivetrain::getSimPose);
        vision.setPoseEstimator(drivetrain.getPoseEstimator());
        vision.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds);
        vision.setHeadingSupplier(drivetrain::getRotation);
        vision.setVisionMeasurementConsumer(drivetrain::addVisionMeasurement);
        vision.setPreciseVisionMeasurementConsumer(drivetrain::addPreciseVisionMeasurement);
    }

    private void configureBindings() {
        Controls.configureControls(0, drivetrain, superstructure);
        Controls.configureDrivingTestingControls(1, drivetrain, superstructure, shotCalculator);

        // Commands.sequence(
        // Commands.runOnce(() -> {
        // ballSimulator.addBall(
        // new BallState(new Translation3d(0, 0, 0),
        // new Translation3d(Math.random() * 5 + 5, 0, Math.random() * 5 + 5),
        // new Rotation3d(),
        // new Translation3d(Math.random() * 50, Math.random() * 50, Math.random() *
        // 50)));
        // }), Commands.waitSeconds(1))
        // .repeatedly().ignoringDisable(true).schedule();
    }

    private void configureTriggers() {
        new Trigger(() -> {
            return drivetrain.getInitialized()
                    && shooterHood.getInitialized()
                    && intake.getInitialized();
        }).onTrue(GlobalStates.INITIALIZED.enableCommand());

        new Trigger(GlobalStates.INITIALIZED::enabled)
                .onTrue(leds.requestStateCommand(LEDState.INITIALIZED_CONFIRM).withTimeout(3));

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.parallel(
                        shooterHood.resetControllersCommand(),
                        intake.resetControllersCommand()).withName("resetControllers"));

        // elevator.isStowed.and(manipulator.hasCoral.negate()).and(RobotModeTriggers.disabled().negate())
        // .whileTrue(leds.requestStateCommand(LEDState.READY_FOR_CORAL));
        // manipulator.hasCoral.and(RobotModeTriggers.disabled().negate())
        // .whileTrue(leds.requestStateCommand(LEDState.HAS_CORAL));
        // manipulator.hasAlgae.and(RobotModeTriggers.disabled().negate())
        // .whileTrue(leds.requestStateCommand(LEDState.HAS_ALGAE));

        // manipulator.hasCoral.whileTrue(arm.useCoralGainsCommand());
        // manipulator.hasAlgae.whileTrue(arm.useAlgaeGainsCommand());

        // superstructure.isReadyToScore
        // .whileTrue(leds.requestStateCommand(LEDState.READY_TO_SCORE));

        // new Trigger(() -> superstructure.getReefSetpoint().orElse(null) ==
        // ReefLevel.L1)
        // .whileTrue(leds.requestStateCommand(LEDState.L1));
        // new Trigger(() -> superstructure.getReefSetpoint().orElse(null) ==
        // ReefLevel.L2)
        // .whileTrue(leds.requestStateCommand(LEDState.L2));
        // new Trigger(() -> superstructure.getReefSetpoint().orElse(null) ==
        // ReefLevel.L3)
        // .whileTrue(leds.requestStateCommand(LEDState.L3));
        // new Trigger(() -> superstructure.getReefSetpoint().orElse(null) ==
        // ReefLevel.L4)
        // .whileTrue(leds.requestStateCommand(LEDState.L4));

        // new Trigger(() -> GlobalStates.LOCK_TO_LINE_OVERRIDE.enabled())
        // .whileTrue(leds.requestStateCommand(LEDState.LOCK_TO_LINE_OVERRIDE));
        // new Trigger(() -> GlobalStates.MANUAL_DRIVING_OVERRIDE.enabled())
        // .whileTrue(leds.requestStateCommand(LEDState.MANUAL_DRIVING_OVERRIDE));
    }

    private void configureAuto() {
        try {
            AutoManager.getInstance().addRoutine(Autos.wheelRadiusCharacterization(drivetrain));
            AutoManager.getInstance().addRoutine(Autos.FDC(drivetrain));
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Could not create trajectories.");
        }

    }
}
