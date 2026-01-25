package frc.robot.subsystems;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlib.swerve.KrakenSwerveDrive;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Controls.*;
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

    /** Lock used for odometry thread. */

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

    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    @Log(groups = "control")
    private boolean isControlledRotationEnabled = false;

    private final SysIdRoutine.Config sysIdConfigDrive = new SysIdRoutine.Config(null, Volts.of(3), null, null);
    private final SysIdRoutine.Config sysIdConfigSteer = new SysIdRoutine.Config();

    public final KrakenSwerveDrive swerve;

    private SwerveModulePosition[] lastModulePositions;

    /**
     * Variable to track the field relative velocities from the previous loop.
     * Used to calculate the acceleration of the robot.
     */
    private ChassisSpeeds prevFieldRelVelocities = new ChassisSpeeds();

    /** Initializes the drivetrain. */
    public Drivetrain(PositionTracker positionTracker) {

        driveController.setTolerance(0.05, 0.05);
        rotationController.setTolerance(0.05, 0.05);
        rotationController.enableContinuousInput(0, 2 * Math.PI);

        swerve = new KrakenSwerveDrive(frontLeft, frontRight, backLeft, backRight,
            pigeon, driveMode, KINEMATICS, SWERVE_CONSTANTS, this,
            sysIdConfigDrive, sysIdConfigSteer, 1);
        
        AutoManager.getInstance().setResetOdometryConsumer(swerve::resetPoseEstimator);

        lastModulePositions = swerve.getModulePositions();
    }

    @Override
    public void periodic() {
        prevFieldRelVelocities = swerve.getFieldRelativeSpeeds();
        swerve.drawRobotOnField(AutoManager.getInstance().getField());
    }

    /**
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        SwerveModulePosition[] currentPositions = swerve.getModulePositions();
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
                thetaSpeed = rotationController.calculate(swerve.getRotation().getRadians());
                // + rotationController.getSetpoint().velocity;
            }

            swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
        }).withName("drivetrain.teleopDrive");
    }

    public Command spinFastCommand() {
        return run(() -> {
            swerve.drive(new ChassisSpeeds(0, 0, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                    DriveMode.FIELD_ORIENTED);
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
                rotationController.reset(swerve.getRotation().getRadians());
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
                rotationController.reset(swerve.getRotation().getRadians());
            }
            if (Utils.shouldFlipValueToRed())
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            else
                rotationController.setGoal(angle.getAsDouble());
        }).andThen(run(() -> {
            swerve.drive(new ChassisSpeeds(0, 0,
                    rotationController.calculate(swerve.getPose().getRotation().getRadians())),
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
            swerve.setStates(new SwerveModuleState[] {
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
            swerve.setStates(new SwerveModuleState[] {
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
                swerve::getPrecisePose,
                swerve::getChassisSpeeds,
                swerve::driveClosedLoop,
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
                this).andThen(runOnce(swerve::stop)).withName("drivetrain.followPath");
    }

    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> followPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                swerve.getPose(), swerve.getPose().plus(delta)),
                        constraints,
                        new IdealStartingState(0, swerve.getRotation()),
                        new GoalEndState(0, delta.getRotation().plus(swerve.getRotation())))),
                Set.of()).withName("drivetrain.driveDelta");
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode).withName("drivetrain.setDriveMode");
    }

    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> swerve.resetGyro()).withName("drivetrain.resetGyro");
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
        return runOnce(swerve::stop)
                .andThen(() -> swerve.setMotorNeutralModes(NeutralModeValue.Coast))
                .finallyDo((d) -> swerve.setMotorNeutralModes(NeutralModeValue.Brake))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("drivetrain.coastMotors");
    }

    @Override
    public Command sysIdDriveQuasistaticCommand(SysIdRoutine.Direction direction) {
        return swerve.getSysIdDrive().quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    @Override
    public Command sysIdDriveDynamicCommand(SysIdRoutine.Direction direction) {
        return swerve.getSysIdDrive().dynamic(direction).withName("drivetrain.sysIdDriveDynamic");
    }

    @Override
    public Command sysIdSteerQuasistaticCommand(SysIdRoutine.Direction direction) {
        return swerve.getSysIdSteer().quasistatic(direction).withName("drivetrain.sysIdSteerQuasistatic");
    }

    @Override
    public Command sysIdSteerDynamicCommand(SysIdRoutine.Direction direction) {
        return swerve.getSysIdSteer().dynamic(direction).withName("drivetrain.sysIdSteerDynamic");
    }

    @Override
    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return runOnce(() -> {
            Translation2d currentPosition = swerve.getPrecisePose().getTranslation();
            Translation2d targetPosition = poseSupplier.get().getTranslation();

            Translation2d toTarget = targetPosition.minus(currentPosition);
            Rotation2d directionToTarget = toTarget.getAngle();

            ChassisSpeeds currentSpeeds = swerve.getFieldRelativeSpeeds();
            Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond);

            double velocityTarget = -currentVelocity.getNorm() * Math.cos(
                    currentVelocity.getAngle().minus(directionToTarget).getRadians());

            driveController.reset(toTarget.getNorm(), velocityTarget < 0 ? velocityTarget : 0);
            rotationController.reset(swerve.getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveController.reset(swerve.getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation()),
                    driveController.getSetpoint().velocity);
            // using one controller (distance from pose) compared to two (x and y distance)
            // so that we move in a straight line and not a curve
            driveToPoseDistance = swerve.getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation());

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
                    + rotationController.calculate(swerve.getPrecisePose().getRotation().getRadians(),
                            poseSupplier.get().getRotation().getRadians());

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    swerve.getPrecisePose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
                    .transformBy(new Transform2d(driveToPoseScalar, 0, Rotation2d.kZero))
                    .getTranslation();

            swerve.driveClosedLoop(
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
            Translation2d currentPosition = swerve.getPrecisePose().getTranslation();
            Translation2d targetPosition = poseSupplier.get().getTranslation();

            Translation2d toTarget = targetPosition.minus(currentPosition);
            Rotation2d directionToTarget = toTarget.getAngle();

            ChassisSpeeds currentSpeeds = swerve.getFieldRelativeSpeeds();
            Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond);

            double velocityTarget = -currentVelocity.getNorm() * Math.cos(
                    currentVelocity.getAngle().minus(directionToTarget).getRadians());

            driveController.reset(toTarget.getNorm(), velocityTarget < 0 ? velocityTarget : 0);
            rotationController.reset(swerve.getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveController.reset(swerve.getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation()),
                    driveController.getSetpoint().velocity);
            // using one controller (distance from pose) compared to two (x and y distance)
            // so that we move in a straight line and not a curve
            driveToPoseDistance = swerve.getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation());

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
                    + rotationController.calculate(swerve.getPrecisePose().getRotation().getRadians(),
                            poseSupplier.get().getRotation().getRadians());

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    swerve.getPrecisePose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
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

            swerve.driveClosedLoop(
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
            Translation2d currentPos = swerve.getPrecisePose().getTranslation();
            Pose2d closestPoseOnLine = Utils.getClosestPoseOnLine(swerve.getPrecisePose(), poseSupplier.get());
            Translation2d closestPos = closestPoseOnLine.getTranslation();

            Rotation2d lineDirection = poseSupplier.get().getRotation();

            Translation2d toLine = closestPos.minus(currentPos);

            Translation2d perpToLine = new Translation2d(-lineDirection.getSin(), lineDirection.getCos());

            double signedPerpendicularDistance = toLine.getX() * perpToLine.getX() + toLine.getY() * perpToLine.getY();

            Translation2d fieldRelativeVelocity = new Translation2d(
                    swerve.getFieldRelativeSpeeds().vxMetersPerSecond,
                    swerve.getFieldRelativeSpeeds().vyMetersPerSecond);

            double signedPerpendicularVelocity = fieldRelativeVelocity.getX() * perpToLine.getX() +
                    fieldRelativeVelocity.getY() * perpToLine.getY();

            driveController.reset(-signedPerpendicularDistance,
                    signedPerpendicularDistance < 0 ? signedPerpendicularVelocity : 0);
            rotationController.reset(swerve.getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            Pose2d pose = poseSupplier.get();
            double cosTheta = pose.getRotation().getCos();
            double sinTheta = pose.getRotation().getSin();

            // formula for distance between point and line given Ax + By + C = 0
            driveToPoseDistance = Utils.getLineDistance(swerve.getPrecisePose(), pose);

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

            double thetaSpeed = rotationController.calculate(swerve.getRotation().getRadians(),
                    pose.getRotation().plus(rotationTransformSupplier.get()).getRadians());

            swerve.driveClosedLoop(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
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
            Translation2d currentPos = swerve.getPrecisePose().getTranslation();
            Pose2d closestPoseOnLine = Utils.getClosestPoseOnLine(swerve.getPrecisePose(), poseSupplier.get());
            Translation2d closestPos = closestPoseOnLine.getTranslation();

            Rotation2d lineDirection = poseSupplier.get().getRotation();

            Translation2d toLine = closestPos.minus(currentPos);

            Translation2d perpToLine = new Translation2d(-lineDirection.getSin(), lineDirection.getCos());

            double signedPerpendicularDistance = toLine.getX() * perpToLine.getX() + toLine.getY() * perpToLine.getY();

            Translation2d fieldRelativeVelocity = new Translation2d(
                    swerve.getFieldRelativeSpeeds().vxMetersPerSecond,
                    swerve.getFieldRelativeSpeeds().vyMetersPerSecond);

            double signedPerpendicularVelocity = fieldRelativeVelocity.getX() * perpToLine.getX() +
                    fieldRelativeVelocity.getY() * perpToLine.getY();

            driveController.reset(-signedPerpendicularDistance,
                    signedPerpendicularVelocity > 0 ? signedPerpendicularVelocity : 0);
            rotationController.reset(swerve.getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            Pose2d pose = poseSupplier.get();
            Rotation2d rotation = pose.getRotation();
            double cosTheta = rotation.getCos();
            double sinTheta = rotation.getSin();

            driveToPoseDistance = Utils.getLineDistance(swerve.getPrecisePose(), pose);

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
                    swerve.getRotation().getRadians(),
                    rotation.plus(rotationTransformSupplier.get()).getRadians());

            swerve.driveClosedLoop(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
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
            Transform2d diff = swerve.getPose().minus(target);
            Rotation2d rot = new Rotation2d(diff.getX(), diff.getY());
            rot = rot.plus(Rotation2d.kPi);
            return rot.getRadians();
        }).withName("drivetrain.targetPose");
    }

    @Log(groups = "control")
    public ChassisAccelerations getFieldRelativeAccelerations() {
        return new ChassisAccelerations(swerve.getFieldRelativeSpeeds(), prevFieldRelVelocities, 0.020);
    }
}