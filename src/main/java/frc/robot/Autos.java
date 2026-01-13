package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;
import com.techhounds.houndutil.houndlib.DoubleContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public class Autos {
    public static AutoRoutine wheelRadiusCharacterization(Drivetrain drivetrain) {
        // wheel radius (meters) = gyro delta (radians) * drive base radius (meters) /
        // wheel position delta (radians)

        DoubleContainer initialGyroRotation = new DoubleContainer(0);
        DoubleContainer initialWheelPosition = new DoubleContainer(0);
        Command command = Commands.sequence(
                drivetrain.teleopDriveCommand(() -> 0, () -> 0, () -> 0.5).alongWith(
                        Commands.waitSeconds(1).andThen(
                                Commands.runOnce(() -> {
                                    initialGyroRotation.value = drivetrain.getRawYaw().in(Radians);
                                    initialWheelPosition.value = Rotations
                                            .of(drivetrain.getModulePositions()[0].distanceMeters
                                                    / Constants.Drivetrain.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE)
                                            .in(Radians);
                                })))
                        .withTimeout(10),

                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    System.out.println("################################################");
                    System.out.println("######## Wheel Characterization Results ########");
                    System.out.println("################################################");

                    double gyroDelta = drivetrain.getRawYaw().in(Radians) - initialGyroRotation.value;
                    double wheelPositionDelta = Math.abs(Rotations.of(drivetrain.getModulePositions()[0].distanceMeters
                            / Constants.Drivetrain.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE).in(Radians)
                            - initialWheelPosition.value);

                    double wheelRadius = (gyroDelta * Constants.Drivetrain.DRIVE_BASE_RADIUS_METERS.in(Meters))
                            / wheelPositionDelta;

                    System.out.println("Gyro Delta (rad): " + gyroDelta);
                    System.out.println("Wheel Position Delta (rad): " + wheelPositionDelta);
                    System.out.println("Wheel Radius (m): " + wheelRadius);
                }));

        return new AutoRoutine("WheelRadius", command, List.of(), Pose2d.kZero);
    }

    public static AutoRoutine FDC(Drivetrain drivetrain)
            throws IOException, ParseException {
        // PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("TestPath", 0);
        // Pose2d startingPose = path.getStartingHolonomicPose().get();

        // Command command = Commands.sequence(
        // drivetrain.followPathCommand(path));

        // Commands.waitSeconds(1).andThen(drivetrain.followPathCommand(pathCToIntake))
        // .alongWith(superstructure.stowAfterScoreCommand()),
        // // Commands.waitSeconds(0.4),
        // Commands.waitUntil(superstructure.manipulator.hasCoral),
        // drivetrain.followPathCommand(pathIntakeToB)
        // .alongWith(Commands.waitSeconds(1.4)
        // .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4, elevator,
        // arm))),
        // superstructure.scoreCoralCommand(manipulator));

        return new AutoRoutine("TestAuto", Commands.none(), List.of(), new Pose2d());
    }
}
