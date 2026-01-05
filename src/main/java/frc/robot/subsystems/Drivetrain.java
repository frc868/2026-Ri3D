package frc.robot.subsystems;

import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Controls.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * The drivetrain subsystem. Manages swerve modules and pose estimation.
 */
@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule frontLeft = new KrakenCoaxialSwerveModule(
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            FRONT_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule frontRight = new KrakenCoaxialSwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            FRONT_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule backLeft = new KrakenCoaxialSwerveModule(
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            BACK_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule backRight = new KrakenCoaxialSwerveModule(
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            BACK_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    @Log
    @SendableLog(name = "pigeonSendable")
    private final Pigeon2 pigeon = new Pigeon2(0, CAN_BUS_NAME);

    private SwerveDriveOdometry simOdometry;
    private SwerveModulePosition[] lastModulePositions = getModulePositions();

    private final MutVoltage sysidDriveAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidDrivePositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidDriveVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdDrive;

    private final MutVoltage sysidSteerAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidSteerPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidSteerVelocityMeasure = RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdSteer;

    private final Orchestra orchestra = new Orchestra();

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDrivePoseEstimator precisePoseEstimator;

    /** Lock used for odometry thread. */
    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();

    private final OdometryThread odometryThread;

    @Log(groups = "control")
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    @Log(groups = "control")
    private double driveToPoseDistance = 0.0;
    @Log(groups = "control")
    private double driveToPoseScalar = 0.0;

    @Log(groups = "control")
    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            THETA_kP, THETA_kI, THETA_kD, THETA_CONSTRAINTS);

    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();
    @Log(groups = "control")
    private ChassisSpeeds adjustedChassisSpeeds = new ChassisSpeeds();

    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    @Log(groups = "control")
    private boolean isControlledRotationEnabled = false;

    /**
     * Local variable to track the field relative velocities from the previous loop.
     * Used to calculate the acceleration of the robot.
     */
    private ChassisSpeeds prevFieldRelVelocities = new ChassisSpeeds();

    private double averageOdometryLoopTime = 0;
    @Log(groups = "odometry")
    // DAQ = data acquisition, from CTRE's odometry thread
    private int successfulDaqs = 0;
    @Log(groups = "odometry")
    private int failedDaqs = 0;

    @Log
    private boolean initialized = RobotBase.isSimulation();

    private final PositionTracker positionTracker;

    /** Initializes the drivetrain. */
    public Drivetrain(PositionTracker positionTracker) {
        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, Rotation2d.kZero));
        precisePoseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, Rotation2d.kZero));

        driveController.setTolerance(0.05, 0.05);
        rotationController.setTolerance(0.05, 0.05);
        rotationController.enableContinuousInput(0, 2 * Math.PI);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        if (RobotBase.isSimulation()) {
            simOdometry = new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions(), new Pose2d());
        }

        sysIdDrive = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drive(new ChassisSpeeds(
                                    SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() / 12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(frontLeft.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(frontLeft.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(frontLeft.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("frontRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(frontRight.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(frontRight.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(frontRight.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("backLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(backLeft.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(backLeft.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(backLeft.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("backRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(backRight.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(backRight.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(backRight.getDriveMotorVelocity(), MetersPerSecond));
                        },
                        this));

        sysIdSteer = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drive(new ChassisSpeeds(
                                    SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() /
                                            12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(frontLeft.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(frontLeft.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(frontLeft.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("frontRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(frontRight.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(frontRight.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(frontRight.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("backLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(backLeft.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(backLeft.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(backLeft.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("backRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(backRight.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(backRight.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(backRight.getSteerMotorVelocity(), RotationsPerSecond));
                        },
                        this));

        orchestra.addInstrument(frontLeft.getDriveMotor(), 1);
        orchestra.addInstrument(frontRight.getDriveMotor(), 1);
        orchestra.addInstrument(backLeft.getDriveMotor(), 1);
        orchestra.addInstrument(backRight.getDriveMotor(), 1);
        orchestra.addInstrument(frontLeft.getSteerMotor(), 1);
        orchestra.addInstrument(frontRight.getSteerMotor(), 1);
        orchestra.addInstrument(backLeft.getSteerMotor(), 1);
        orchestra.addInstrument(backRight.getSteerMotor(), 1);

        odometryThread = new OdometryThread();
        odometryThread.start();

        this.positionTracker = positionTracker;

    }

    /**
     * Thread enabling 250Hz odometry. Optimized from CTRE's internal swerve code.
     * 250Hz odometry reduces discretization error in the odometry loop, and
     * significantly improves odometry during high speed maneuvers.
     */
    public class OdometryThread {
        // Testing shows 1 (minimum realtime) is sufficient for tighter odometry loops.
        // If the odometry period is far away from the desired frequency, increasing
        // this may help
        private static final int START_THREAD_PRIORITY = 1;

        private final Thread m_thread;
        private volatile boolean m_running = false;

        private final BaseStatusSignal[] allSignals;

        private final MedianFilter peakRemover = new MedianFilter(3);
        private final LinearFilter lowPass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;

        private KrakenCoaxialSwerveModule[] modules = new KrakenCoaxialSwerveModule[] {
                frontLeft, frontRight, backLeft, backRight };
        private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        private int lastThreadPriority = START_THREAD_PRIORITY;
        private volatile int threadPriorityToSet = START_THREAD_PRIORITY;
        private final int UPDATE_FREQUENCY = 250;

        public OdometryThread() {
            m_thread = new Thread(this::run);
            /*
             * Mark this thread as a "daemon" (background) thread
             * so it doesn't hold up program shutdown
             */
            m_thread.setDaemon(true);

            /* 4 signals for each module + 2 for Pigeon2 */

            allSignals = new BaseStatusSignal[(4 * 4) + 2];
            for (int i = 0; i < 4; ++i) {
                BaseStatusSignal[] signals = modules[i].getSignals();
                allSignals[(i * 4) + 0] = signals[0];
                allSignals[(i * 4) + 1] = signals[1];
                allSignals[(i * 4) + 2] = signals[2];
                allSignals[(i * 4) + 3] = signals[3];
            }
            allSignals[allSignals.length - 2] = pigeon.getYaw();
            allSignals[allSignals.length - 1] = pigeon.getAngularVelocityZWorld();
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            m_running = true;
            m_thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         *
         * @param millis The time to wait in milliseconds
         */
        public void stop(long millis) {
            m_running = false;
            try {
                m_thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        public void run() {
            /* Make sure all signals update at the correct update frequency */
            BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY, allSignals);
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

            /* Run as fast as possible, our signals will control the timing */
            while (m_running) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                status = BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQUENCY, allSignals);

                try {
                    stateLock.writeLock().lock();

                    lastTime = currentTime;
                    currentTime = RobotController.getFPGATime();
                    /*
                     * We don't care about the peaks, as they correspond to GC events, and we want
                     * the period generally low passed
                     */
                    averageOdometryLoopTime = lowPass.calculate(peakRemover.calculate((currentTime - lastTime) / 1000));

                    /* Get status of first element */
                    if (status.isOK()) {
                        successfulDaqs++;
                    } else {
                        failedDaqs++;
                    }

                    /* Now update odometry */
                    /* Keep track of the change in azimuth rotations */
                    for (int i = 0; i < 4; ++i) {
                        modulePositions[i] = modules[i].getPosition();
                    }
                    double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                            pigeon.getYaw(), pigeon.getAngularVelocityZWorld()).magnitude();

                    /* Keep track of previous and current pose to account for the carpet vector */
                    poseEstimator.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
                    precisePoseEstimator.update(Rotation2d.fromDegrees(yawDegrees),
                            modulePositions);
                    if (RobotBase.isSimulation()) {
                        simOdometry.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
                    }
                } finally {
                    stateLock.writeLock().unlock();
                }

                /**
                 * This is inherently synchronous, since lastThreadPriority
                 * is only written here and threadPriorityToSet is only read here
                 */
                if (threadPriorityToSet != lastThreadPriority) {
                    Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                    lastThreadPriority = threadPriorityToSet;
                }
            }
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified
         * priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher
         *                 priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            threadPriorityToSet = priority;
        }
    }

    public double getOdometryLoopTime() {
        return averageOdometryLoopTime;
    }

    @Override
    public void periodic() {
        prevFieldRelVelocities = getFieldRelativeSpeeds();
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    /**
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        SwerveModulePosition[] currentPositions = getModulePositions();
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            deltas[i] = new SwerveModulePosition(
                    currentPositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    currentPositions[i].angle);
        }

        pigeon.getSimState().setRawYaw(pigeon.getYaw().getValueAsDouble() +
                Units.radiansToDegrees(KINEMATICS.toTwist2d(deltas).dtheta));

        lastModulePositions = currentPositions;
    }

    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    @Override
    @Log
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Log
    public Pose2d getPrecisePose() {
        return precisePoseEstimator.getEstimatedPosition();
    }

    public Pose2d getSimPose() {
        if (simOdometry != null)
            return simOdometry.getPoseMeters();
        else
            return new Pose2d();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    public Angle getRawYaw() {
        return pigeon.getYaw().getValue();
    }

    @Override
    @Log(groups = "control")
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    @Log(groups = "control")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    @Log(groups = "control")
    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @Log(groups = "control")
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(KINEMATICS.toChassisSpeeds(getModuleStates()), getRotation());
    }

    @Log(groups = "control")
    public ChassisAccelerations getFieldRelativeAccelerations() {
        return new ChassisAccelerations(getFieldRelativeSpeeds(), prevFieldRelVelocities, 0.020);
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Adds a vision measurement to the pose estimator. Used by the vision
     * subsystem.
     * 
     * @param visionRobotPoseMeters    the estimated robot pose from vision
     * @param timestampSeconds         the timestamp of the vision measurement
     * @param visionMeasurementStdDevs the standard deviations of the measurement
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // since the pose estimator is used by another thread, we need to lock it to be
            // able to add a vision measurement
            stateLock.writeLock().lock();
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            stateLock.writeLock().unlock();
        }

    }

    /**
     * Adds a precise vision measurement to the pose estimator. Used by the vision
     * subsystem.
     * 
     * @param visionRobotPoseMeters    the estimated robot pose from vision
     * @param timestampSeconds         the timestamp of the vision measurement
     * @param visionMeasurementStdDevs the standard deviations of the measurement
     */
    public void addPreciseVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // since the pose estimator is used by another thread, we need to lock it to be
            // able to add a vision measurement
            stateLock.writeLock().lock();
            precisePoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
                    visionMeasurementStdDevs);
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    @Override
    public void updatePoseEstimator() {
        // unused due to multi-threaded odometry
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        try {
            // since the pose estimator is used by another thread, we need to lock it to be
            // able to reset it
            stateLock.writeLock().lock();

            poseEstimator.resetPosition(getRotation(), getModulePositions(),
                    new Pose2d(pose.getTranslation(), getRotation()));
            precisePoseEstimator.resetPosition(getRotation(), getModulePositions(),
                    new Pose2d(pose.getTranslation(), getRotation()));
            if (RobotBase.isSimulation())
                simOdometry.resetPosition(getRotation(), getModulePositions(),
                        new Pose2d(pose.getTranslation(), getRotation()));
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    @Override
    public void resetGyro() {
        pigeon.setYaw(0);
        initialized = true;
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        frontLeft.setMotorHoldMode(motorHoldMode);
        frontRight.setMotorHoldMode(motorHoldMode);
        backLeft.setMotorHoldMode(motorHoldMode);
        backRight.setMotorHoldMode(motorHoldMode);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void setStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void setStatesClosedLoop(SwerveModuleState[] states) {
        frontLeft.setStateClosedLoop(states[0]);
        frontRight.setStateClosedLoop(states[1]);
        backLeft.setStateClosedLoop(states[2]);
        backRight.setStateClosedLoop(states[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, this.driveMode);
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        if (Utils.shouldFlipValueToRed()
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        // compensates for swerve skew when translating and rotating simultaneously
        adjustedChassisSpeeds = ChassisSpeeds.discretize(adjustedChassisSpeeds, 0.02);
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states[0].optimize(frontLeft.getWheelAngle());
        states[1].optimize(frontRight.getWheelAngle());
        states[2].optimize(backLeft.getWheelAngle());
        states[3].optimize(backRight.getWheelAngle());

        commandedModuleStates = states;
        setStates(states);
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        if (Utils.shouldFlipValueToRed()
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        adjustedChassisSpeeds = ChassisSpeeds.discretize(adjustedChassisSpeeds, 0.02);
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states[0].optimize(frontLeft.getWheelAngle());
        states[1].optimize(frontRight.getWheelAngle());
        states[2].optimize(backLeft.getWheelAngle());
        states[3].optimize(backRight.getWheelAngle());

        commandedModuleStates = states;
        setStatesClosedLoop(states);
    }

    public void driveClosedLoop(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        // TODO implement
        driveClosedLoop(speeds, DriveMode.ROBOT_RELATIVE);
    }

    @Override
    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, JOYSTICK_INPUT_DEADBAND);
            ySpeed = MathUtil.applyDeadband(ySpeed, JOYSTICK_INPUT_DEADBAND);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, JOYSTICK_INPUT_DEADBAND);

            xSpeed = Math.copySign(Math.pow(xSpeed, JOYSTICK_CURVE_EXP), xSpeed);
            ySpeed = Math.copySign(Math.pow(ySpeed, JOYSTICK_CURVE_EXP), ySpeed);
            thetaSpeed = Math.copySign(Math.pow(thetaSpeed, JOYSTICK_ROT_CURVE_EXP), thetaSpeed);

            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);
            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            ySpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5;

            if (isControlledRotationEnabled) {
                thetaSpeed = rotationController.calculate(getRotation().getRadians());
            }

            drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
        }).withName("drivetrain.teleopDrive");
    }

    public Command spinFastCommand() {
        return run(() -> {
            drive(new ChassisSpeeds(0, 0, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND), DriveMode.FIELD_ORIENTED);
        }).withName("drivetrain.teleopDrive");
    }

    /**
     * Command to enable rotation to a specific angle while driving (controller used
     * in {@link #drive}).
     * 
     * @param angle the angle to rotate to
     * @return the command
     */
    @Override
    public Command controlledRotateCommand(DoubleSupplier angle) {
        return Commands.run(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(getRotation().getRadians());
            }
            isControlledRotationEnabled = true;
            if (Utils.shouldFlipValueToRed())
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            else
                rotationController.setGoal(angle.getAsDouble());
        }).withName("drivetrain.controlledRotate");
    }

    /**
     * Command to take over drivetrain control and rotate the chassis to a specific
     * angle.
     * 
     * @param angle the angle to rotate to
     * @return the command
     */
    public Command standaloneControlledRotateCommand(DoubleSupplier angle) {
        return runOnce(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(getRotation().getRadians());
            }
            if (Utils.shouldFlipValueToRed())
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            else
                rotationController.setGoal(angle.getAsDouble());
        }).andThen(run(() -> {
            drive(new ChassisSpeeds(0, 0,
                    rotationController.calculate(poseEstimator.getEstimatedPosition().getRotation().getRadians())),
                    driveMode);
        })).withName("drivetrain.standaloneControlledRotate");
    }

    @Override
    public Command disableControlledRotateCommand() {
        return Commands.runOnce(
                () -> {
                    isControlledRotationEnabled = false;
                }).withName("drivetrain.disableControlledRotate");
    }

    public Command enableControlledRotateCommand() {
        return Commands.runOnce(
                () -> {
                    isControlledRotationEnabled = true;
                }).withName("drivetrain.enableControlledRotate");
    }

    @Override
    public Command wheelLockCommand() {
        return run(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("drivetrain.wheelLock");
    }

    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        return Commands.runOnce(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        }).withName("drivetrain.turnWheelsToAngle");
    }

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                this::getPrecisePose,
                this::getChassisSpeeds,
                this::driveClosedLoop,
                new PPHolonomicDriveController(
                        new PIDConstants(PATH_FOLLOWING_TRANSLATION_kP, 0, 0),
                        new PIDConstants(PATH_FOLLOWING_ROTATION_kP, 0, 0)),
                ROBOT_CONFIG,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this).andThen(runOnce(this::stop)).withName("drivetrain.followPath");
    }

    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> followPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                getPose(), getPose().plus(delta)),
                        constraints,
                        new IdealStartingState(0, getRotation()),
                        new GoalEndState(0, delta.getRotation().plus(getRotation())))),
                Set.of()).withName("drivetrain.driveDelta");
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode).withName("drivetrain.setDriveMode");
    }

    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> resetGyro()).withName("drivetrain.resetGyro");
    }

    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        return Commands.runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        }).withName("drivetrain.setDriveCurrentLimit");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(this::stop)
                .andThen(() -> setMotorHoldModes(MotorHoldMode.COAST))
                .finallyDo((d) -> setMotorHoldModes(MotorHoldMode.BRAKE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("drivetrain.coastMotors");
    }

    /**
     * Draws the robot on a Field2d. This will include the angles of the swerve
     * modules on the outsides of the robot box in Glass.
     * 
     * @param field the field to draw the robot on (usually
     *              {@code AutoManager.getInstance().getField()})
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        if (RobotBase.isSimulation())
            field.getObject("simPose").setPose(simOdometry.getPoseMeters());
        field.getObject("precisePose").setPose(precisePoseEstimator.getEstimatedPosition());
    }

    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdDrive.quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return sysIdDrive.dynamic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdSteer.quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return sysIdSteer.dynamic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command manualInitializeCommand() {
        return Commands.runOnce(() -> this.initialized = true).ignoringDisable(true)
                .withName("drivetrain.manualInitialize");
    }

    @Log
    public boolean getInitialized() {
        return initialized;
    }

    @Override
    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return runOnce(() -> {
            Translation2d currentPosition = getPrecisePose().getTranslation();
            Translation2d targetPosition = poseSupplier.get().getTranslation();

            Translation2d toTarget = targetPosition.minus(currentPosition);
            Rotation2d directionToTarget = toTarget.getAngle();

            ChassisSpeeds currentSpeeds = getFieldRelativeSpeeds();
            Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond);

            double velocityTarget = -currentVelocity.getNorm() * Math.cos(
                    currentVelocity.getAngle().minus(directionToTarget).getRadians());

            driveController.reset(toTarget.getNorm(), velocityTarget < 0 ? velocityTarget : 0);
            rotationController.reset(getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveController.reset(getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation()),
                    driveController.getSetpoint().velocity);
            // using one controller (distance from pose) compared to two (x and y distance)
            // so that we move in a straight line and not a curve
            driveToPoseDistance = getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation());

            // prevents fast swapping of feedforward back and forth around setpoint
            double ffScaler = MathUtil.clamp(
                    (driveToPoseDistance - XY_FF_MIN_RANGE) / (XY_FF_MAX_RANGE - XY_FF_MIN_RANGE),
                    0.0,
                    1.0);

            driveToPoseScalar = driveController.getSetpoint().velocity * ffScaler + driveController.calculate(
                    driveToPoseDistance, 0.0);
            if (Utils.shouldFlipValueToRed()) {
                driveToPoseScalar *= -1;
            }
            if (driveController.atGoal())
                driveToPoseScalar = 0.0;

            double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler
                    + rotationController.calculate(getPrecisePose().getRotation().getRadians(),
                            poseSupplier.get().getRotation().getRadians());

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    getPrecisePose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
                    .transformBy(new Transform2d(driveToPoseScalar, 0, Rotation2d.kZero))
                    .getTranslation();

            driveClosedLoop(
                    new ChassisSpeeds(
                            driveVelocity.getX(),
                            driveVelocity.getY(),
                            thetaVelocity),
                    DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.driveToPose");
    }

    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier,
            Supplier<TrapezoidProfile.Constraints> constraintsSupplier) {
        return Commands.runOnce(() -> driveController.setConstraints(constraintsSupplier.get()))
                .andThen(driveToPoseCommand(poseSupplier));
    }

    public Command driveToPoseAdaptiveCommand(Supplier<Pose2d> poseSupplier, DoubleSupplier xJoystickSupplier,
            DoubleSupplier yJoystickSupplier, DoubleSupplier thetaJoystickSupplier) {
        return runOnce(() -> {
            Translation2d currentPosition = getPrecisePose().getTranslation();
            Translation2d targetPosition = poseSupplier.get().getTranslation();

            Translation2d toTarget = targetPosition.minus(currentPosition);
            Rotation2d directionToTarget = toTarget.getAngle();

            ChassisSpeeds currentSpeeds = getFieldRelativeSpeeds();
            Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond);

            double velocityTarget = -currentVelocity.getNorm() * Math.cos(
                    currentVelocity.getAngle().minus(directionToTarget).getRadians());

            driveController.reset(toTarget.getNorm(), velocityTarget < 0 ? velocityTarget : 0);
            rotationController.reset(getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveController.reset(getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation()),
                    driveController.getSetpoint().velocity);
            // using one controller (distance from pose) compared to two (x and y distance)
            // so that we move in a straight line and not a curve
            driveToPoseDistance = getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation());

            // prevents fast swapping of feedforward back and forth around setpoint
            double ffScaler = MathUtil.clamp(
                    (driveToPoseDistance - XY_FF_MIN_RANGE) / (XY_FF_MAX_RANGE - XY_FF_MIN_RANGE),
                    0.0,
                    1.0);

            driveToPoseScalar = driveController.getSetpoint().velocity * ffScaler + driveController.calculate(
                    driveToPoseDistance, 0.0);
            if (Utils.shouldFlipValueToRed()) {
                driveToPoseScalar *= -1;
            }
            if (driveController.atGoal())
                driveToPoseScalar = 0.0;

            double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler
                    + rotationController.calculate(getPrecisePose().getRotation().getRadians(),
                            poseSupplier.get().getRotation().getRadians());

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    getPrecisePose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
                    .transformBy(new Transform2d(driveToPoseScalar, 0, Rotation2d.kZero))
                    .getTranslation();

            Translation2d linearFF = new Translation2d(xJoystickSupplier.getAsDouble(),
                    yJoystickSupplier.getAsDouble());
            final double linearS = linearFF.getNorm() * AUTO_DRIVE_ADAPTIVE_SCALE_FACTOR;
            final double thetaS = Math.abs(thetaJoystickSupplier.getAsDouble()); // [0-1]
            driveVelocity = driveVelocity
                    .interpolate(linearFF.times(1.5), linearS);
            thetaVelocity = MathUtil.interpolate(
                    thetaVelocity, thetaJoystickSupplier.getAsDouble() * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    thetaS);

            driveClosedLoop(
                    new ChassisSpeeds(
                            driveVelocity.getX(),
                            driveVelocity.getY(),
                            thetaVelocity),
                    DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.driveToPoseAdaptive");
    }

    public Command lockToLineCommand(Supplier<Pose2d> poseSupplier, Supplier<Rotation2d> rotationTransformSupplier,
            DoubleSupplier xJoystickSupplier,
            DoubleSupplier yJoystickSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return runOnce(() -> {
            Translation2d currentPos = getPrecisePose().getTranslation();
            Pose2d closestPoseOnLine = Utils.getClosestPoseOnLine(getPrecisePose(), poseSupplier.get());
            Translation2d closestPos = closestPoseOnLine.getTranslation();

            Rotation2d lineDirection = poseSupplier.get().getRotation();

            Translation2d toLine = closestPos.minus(currentPos);

            Translation2d perpToLine = new Translation2d(-lineDirection.getSin(), lineDirection.getCos());

            double signedPerpendicularDistance = toLine.getX() * perpToLine.getX() + toLine.getY() * perpToLine.getY();

            Translation2d fieldRelativeVelocity = new Translation2d(
                    getFieldRelativeSpeeds().vxMetersPerSecond,
                    getFieldRelativeSpeeds().vyMetersPerSecond);

            double signedPerpendicularVelocity = fieldRelativeVelocity.getX() * perpToLine.getX() +
                    fieldRelativeVelocity.getY() * perpToLine.getY();

            driveController.reset(-signedPerpendicularDistance,
                    signedPerpendicularDistance < 0 ? signedPerpendicularVelocity : 0);
            rotationController.reset(getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            Pose2d pose = poseSupplier.get();
            double cosTheta = pose.getRotation().getCos();
            double sinTheta = pose.getRotation().getSin();

            // formula for distance between point and line given Ax + By + C = 0
            driveToPoseDistance = Utils.getLineDistance(getPrecisePose(), pose);

            double ySpeedRelStage = driveController.calculate(driveToPoseDistance, 0.0)
                    + driveController.getSetpoint().velocity;

            double xJoystick = xJoystickSupplier.getAsDouble();
            xJoystick = MathUtil.applyDeadband(xJoystick, JOYSTICK_INPUT_DEADBAND);
            xJoystick = Math.copySign(Math.pow(xJoystick, JOYSTICK_CURVE_EXP), xJoystick);
            xJoystick = xSpeedLimiter.calculate(xJoystick);
            xJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red) {
                xJoystick *= -1;
            }

            double yJoystick = yJoystickSupplier.getAsDouble();
            yJoystick = MathUtil.applyDeadband(yJoystick, JOYSTICK_INPUT_DEADBAND);
            yJoystick = Math.copySign(Math.pow(yJoystick, JOYSTICK_CURVE_EXP), yJoystick);
            yJoystick = ySpeedLimiter.calculate(yJoystick);
            yJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red) {
                yJoystick *= -1;
            }

            // transforms the vector created by the joystick input to the line created by
            // extending the ray from the trap to the edge of the field
            double lineDirX = Math.cos(pose.getRotation().getRadians());
            double lineDirY = Math.sin(pose.getRotation().getRadians());

            double xJoystickSpeedRelStage = xJoystick * lineDirX + yJoystick * lineDirY;

            double xSpeed = xJoystickSpeedRelStage * cosTheta - ySpeedRelStage * sinTheta;
            double ySpeed = xJoystickSpeedRelStage * sinTheta + ySpeedRelStage * cosTheta;

            if (Utils.shouldFlipValueToRed()) {
                xSpeed *= -1;
                ySpeed *= -1;
            }

            double thetaSpeed = rotationController.calculate(getRotation().getRadians(),
                    pose.getRotation().plus(rotationTransformSupplier.get()).getRadians());

            driveClosedLoop(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.lockToLine");
    }

    public Command lockToLineAdaptiveCommand(Supplier<Pose2d> poseSupplier,
            Supplier<Rotation2d> rotationTransformSupplier,
            DoubleSupplier xJoystickSupplier,
            DoubleSupplier yJoystickSupplier,
            DoubleSupplier thetaJoystickSupplier) {

        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return runOnce(() -> {
            Translation2d currentPos = getPrecisePose().getTranslation();
            Pose2d closestPoseOnLine = Utils.getClosestPoseOnLine(getPrecisePose(), poseSupplier.get());
            Translation2d closestPos = closestPoseOnLine.getTranslation();

            Rotation2d lineDirection = poseSupplier.get().getRotation();

            Translation2d toLine = closestPos.minus(currentPos);

            Translation2d perpToLine = new Translation2d(-lineDirection.getSin(), lineDirection.getCos());

            double signedPerpendicularDistance = toLine.getX() * perpToLine.getX() + toLine.getY() * perpToLine.getY();

            Translation2d fieldRelativeVelocity = new Translation2d(
                    getFieldRelativeSpeeds().vxMetersPerSecond,
                    getFieldRelativeSpeeds().vyMetersPerSecond);

            double signedPerpendicularVelocity = fieldRelativeVelocity.getX() * perpToLine.getX() +
                    fieldRelativeVelocity.getY() * perpToLine.getY();

            driveController.reset(-signedPerpendicularDistance,
                    signedPerpendicularVelocity > 0 ? signedPerpendicularVelocity : 0);
            rotationController.reset(getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            Pose2d pose = poseSupplier.get();
            Rotation2d rotation = pose.getRotation();
            double cosTheta = rotation.getCos();
            double sinTheta = rotation.getSin();

            driveToPoseDistance = Utils.getLineDistance(getPrecisePose(), pose);

            // Process joystick input
            double xJoystick = xJoystickSupplier.getAsDouble();
            xJoystick = MathUtil.applyDeadband(xJoystick, JOYSTICK_INPUT_DEADBAND);
            xJoystick = Math.copySign(Math.pow(xJoystick, JOYSTICK_CURVE_EXP), xJoystick);
            xJoystick = xSpeedLimiter.calculate(xJoystick);
            xJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;

            double yJoystick = yJoystickSupplier.getAsDouble();
            yJoystick = MathUtil.applyDeadband(yJoystick, JOYSTICK_INPUT_DEADBAND);
            yJoystick = Math.copySign(Math.pow(yJoystick, JOYSTICK_CURVE_EXP), yJoystick);
            yJoystick = ySpeedLimiter.calculate(yJoystick);
            yJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;

            if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red) {
                xJoystick *= -1;
                yJoystick *= -1;
            }

            // Joystick influence in direction perpendicular to the line
            // Perpendicular to (cosTheta, sinTheta) is (-sinTheta, cosTheta)
            double joystickPerpendicularInfluence = xJoystick * -sinTheta + yJoystick * cosTheta;
            double yJoystickControlledAxisOffset = joystickPerpendicularInfluence * 0.2;

            // PID controller + small driver override
            double ySpeedRelStage = driveController.calculate(driveToPoseDistance, 0.0)
                    + driveController.getSetpoint().velocity
                    + yJoystickControlledAxisOffset;

            // Joystick input projected onto direction of the line (tangent)
            double joystickAlongLine = xJoystick * cosTheta + yJoystick * sinTheta;

            // Transform stage-relative speeds into field-relative
            double xSpeed = joystickAlongLine * cosTheta - ySpeedRelStage * sinTheta;
            double ySpeed = joystickAlongLine * sinTheta + ySpeedRelStage * cosTheta;

            if (Utils.shouldFlipValueToRed()) {
                xSpeed *= -1;
                ySpeed *= -1;
            }

            double thetaSpeed = rotationController.calculate(
                    getRotation().getRadians(),
                    rotation.plus(rotationTransformSupplier.get()).getRadians());

            driveClosedLoop(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.lockToLine");
    }

    /**
     * Creates a command that controls the chassis rotation to keep it pointed a
     * specific target location.
     * 
     * @param targetPose a supplier for the target pose to point the chassis at
     * @return the command
     */
    public Command targetPoseCommand(Supplier<Pose3d> targetPose) {
        return controlledRotateCommand(() -> {
            Pose2d target = targetPose.get().toPose2d();
            Transform2d diff = getPose().minus(target);
            Rotation2d rot = new Rotation2d(diff.getX(), diff.getY());
            rot = rot.plus(Rotation2d.kPi);
            return rot.getRadians();
        }).withName("drivetrain.targetPose");
    }

}