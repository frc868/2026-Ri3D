package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

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
                                    initialGyroRotation.value = drivetrain.swerve.getRawYaw().in(Radians);
                                    initialWheelPosition.value = Rotations
                                            .of(drivetrain.swerve.getModulePositions()[0].distanceMeters
                                                    / Constants.Drivetrain.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE)
                                            .in(Radians);
                                })))
                        .withTimeout(10),

                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    System.out.println("################################################");
                    System.out.println("######## Wheel Characterization Results ########");
                    System.out.println("################################################");

                    double gyroDelta = drivetrain.swerve.getRawYaw().in(Radians) - initialGyroRotation.value;
                    double wheelPositionDelta = Math.abs(Rotations.of(drivetrain.swerve.getModulePositions()[0].distanceMeters
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

    public static AutoRoutine FDC(Drivetrain drivetrain) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'FDC'");
    }
}
