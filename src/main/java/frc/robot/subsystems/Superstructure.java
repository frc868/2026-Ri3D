package frc.robot.subsystems;

import com.techhounds.houndutil.houndlib.BallSimulator;
import com.techhounds.houndutil.houndlib.BallState;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.Tunable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;

import static frc.robot.Constants.Shooter.BALL_TRANSFORM_LEFT;
import static frc.robot.Constants.Shooter.BALL_TRANSFORM_RIGHT;
import static frc.robot.Constants.Shooter.DISTANCE_TO_RPS;
import static frc.robot.Constants.Shooter.DISTANCE_TO_SHOT_SPEED;
import static frc.robot.Constants.ShooterHood.DISTANCE_TO_HOOD_ANGLE;
import java.util.function.Supplier;

@LoggedObject
public class Superstructure extends SubsystemBase {
    private final Drivetrain drivetrain;
    public final Shooter shooter;
    public final ShooterHood shooterHood;
    public final Hopper hopper;
    public final Intake intake;
    public final LEDs leds;

    private final BallSimulator ballSimulator;
    public final ShotCalculator shotCalculator;

    @Tunable
    @Log
    private double testShotVelocity = 0;

    @Tunable
    @Log
    private double testLaunchPitch = 0;

    private final Supplier<Double> distanceSupplier;
    private final Supplier<Double> distanceSotmSupplier;

    public Superstructure(Drivetrain drivetrain, Shooter shooter, ShooterHood shooterHood, Hopper hopper, Intake intake,
            LEDs leds,
            BallSimulator ballSimulator, ShotCalculator shotCalculator) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.shooterHood = shooterHood;
        this.hopper = hopper;
        this.intake = intake;
        this.leds = leds;

        this.ballSimulator = ballSimulator;
        this.shotCalculator = shotCalculator;
        distanceSupplier = () -> drivetrain.getPose().getTranslation()
                .getDistance(FieldConstants.Hub.CENTER.getTranslation().toTranslation2d());
        distanceSotmSupplier = () -> drivetrain.getPose().getTranslation()
                .getDistance(shotCalculator.getCurrentEffectiveTargetPose().getTranslation().toTranslation2d());
    }

    @Override
    public void periodic() {
        ballSimulator.update();
    }

    public Command useRequirement() {
        return runOnce(() -> {
        });
    }

    public Command targetHubEasyCommand() {
        return Commands.parallel(
                shooter.spinAtVelocityCommand(
                        () -> DISTANCE_TO_RPS.get(distanceSupplier.get())).asProxy(),
                shooterHood.moveToArbitraryPositionCommand(
                        () -> DISTANCE_TO_HOOD_ANGLE.get(distanceSupplier.get())).asProxy())
                .andThen(useRequirement());
    }

    public Command targetHubSotmCommand() {
        return Commands.parallel(
                shooter.spinAtVelocityCommand(
                        () -> DISTANCE_TO_RPS.get(distanceSotmSupplier.get())).asProxy(),
                shooterHood.moveToArbitraryPositionCommand(
                        () -> DISTANCE_TO_HOOD_ANGLE.get(distanceSotmSupplier.get())).asProxy())
                .andThen(useRequirement());
    }

    // public Command shootCommand(Drivetrain drivetrain) {
    // return runOnce(() -> {
    // InterceptSolution sol = shotCalculator.getInterceptSolution();
    // shootBall(drivetrain.getPose(), drivetrain.getChassisSpeeds(),
    // sol.launchSpeed(), sol.launchPitchRad());

    // // Pose2d dtPose = drivetrain.getPose();
    // // shootBall(new Pose2d(dtPose.getX(), dtPose.getY(), new
    // // Rotation2d(sol.requiredYaw())),
    // // drivetrain.getChassisSpeeds(), sol.launchSpeed(), sol.launchPitchRad());

    // });
    // }

    public Command testShotCommand(Drivetrain drivetrain) {
        return runOnce(() -> {
            Pose2d drivetrainPose = drivetrain.getPose();
            ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldRelativeSpeeds();

            shootBall(drivetrainPose, drivetrainSpeeds, testShotVelocity, Units.degreesToRadians(testLaunchPitch));
        });
    }

    public void shootBall(Pose2d drivetrainPose, ChassisSpeeds fieldRelChassisSpeeds, double shotVelocity,
            double launchPitch) {

        Translation3d drivetrainVeloTransform = new Translation3d(fieldRelChassisSpeeds.vxMetersPerSecond,
                fieldRelChassisSpeeds.vyMetersPerSecond, 0);

        ballSimulator
                .addBall(new BallState(
                        new Pose3d(new Translation3d(drivetrainPose.getX(), drivetrainPose.getY(), 0),
                                new Rotation3d(drivetrainPose.getRotation())).plus(BALL_TRANSFORM_LEFT),
                        new Translation3d(
                                shotVelocity * Math.cos(launchPitch),
                                0,
                                shotVelocity * Math.sin(launchPitch))
                                .rotateBy(new Rotation3d(drivetrainPose.getRotation()))
                                .plus(drivetrainVeloTransform),
                        new Translation3d(0, 100, 0)));
        ballSimulator
                .addBall(new BallState(
                        new Pose3d(new Translation3d(drivetrainPose.getX(), drivetrainPose.getY(), 0),
                                new Rotation3d(drivetrainPose.getRotation())).plus(BALL_TRANSFORM_RIGHT),
                        new Translation3d(
                                shotVelocity * Math.cos(launchPitch),
                                0,
                                shotVelocity * Math.sin(launchPitch))
                                .rotateBy(new Rotation3d(drivetrainPose.getRotation()))
                                .plus(drivetrainVeloTransform),
                        new Translation3d(0, 100, 0)));
    }

}