package frc.robot;

import java.util.List;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;
import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;
import com.techhounds.houndutil.houndlog.loggers.MetadataLogger.MetadataRecord;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.LEDs.LEDState;

public class Constants {
    public static final double LOOP_TIME = 0.020; // 20ms

    public static final MetadataRecord BUILD_METADATA = new MetadataRecord(
            BuildConstants.MAVEN_NAME, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE,
            BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);

    public static final String CAN_BUS_NAME = "canivore";

    public static final class Drivetrain {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 8;

        public static final int FRONT_LEFT_STEER_ENCODER_ID = 0;
        public static final int FRONT_RIGHT_STEER_ENCODER_ID = 1;
        public static final int BACK_LEFT_STEER_ENCODER_ID = 2;
        public static final int BACK_RIGHT_STEER_ENCODER_ID = 3;

        public static final int PIGEON_ID = 0;

        public static final TunableDouble DEMO_SPEED = new TunableDouble("subsystems/drivetrain/DEMO_SPEED", 1.0);

        public static final boolean DRIVE_MOTORS_INVERTED = false;
        public static final boolean STEER_MOTORS_INVERTED = true;
        public static final boolean STEER_CANCODERS_INVERTED = RobotBase.isReal() ? false : true;

        public static final double FRONT_LEFT_OFFSET = 0; // TODO simvalue
        public static final double FRONT_RIGHT_OFFSET = 0; // TODO simvalue
        public static final double BACK_LEFT_OFFSET = 0; // TODO simvalue
        public static final double BACK_RIGHT_OFFSET = 0; // TODO simvalue

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.60325; // TODO simvalue
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.60325; // TODO simvalue
        /** Distance between the center of the robot and the */
        public static final double DRIVE_BASE_RADIUS_METERS = 0.4265621658; // TODO simvalue
        public static final double MASS_KG = Units.lbsToKilograms(135); // TODO simvalue
        public static final double MOI = 6.0;
        public static final double WHEEL_RADIUS_METERS = 0.05; // TODO simvalue
        public static final double WHEEL_COF = 1.3; // TODO this is a good guess for TPU

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 1.3;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.14609;
            SWERVE_CONSTANTS.DRIVE_kV = 0.73522;
            SWERVE_CONSTANTS.DRIVE_kA = 0.096345;
            SWERVE_CONSTANTS.STEER_kP = 100.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 1.0;
            SWERVE_CONSTANTS.STEER_kS = 0;
            SWERVE_CONSTANTS.STEER_kV = 0;
            SWERVE_CONSTANTS.STEER_kA = 0;

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.14;
            SWERVE_CONSTANTS.STEER_GEARING = 24.0 / 1.0;
            SWERVE_CONSTANTS.COUPLING_RATIO = 50.0 / 13.0; // Thrifty Narrow
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = RobotBase.isReal() ? 4.93 : 5.29; // TODO simvalue
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.41; // TODO simvalue
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 100 * 2 * Math.PI;
            // max velocity in 1/10 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 10 * 100 * 2 * Math.PI;

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 85;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 30;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.01;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 11.47; // TODO simvalue
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 30; // TODO simvalue

        public static final Translation2d[] SWERVE_MODULE_LOCATIONS = new Translation2d[] {
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2) };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                SWERVE_MODULE_LOCATIONS[0],
                SWERVE_MODULE_LOCATIONS[1],
                SWERVE_MODULE_LOCATIONS[2],
                SWERVE_MODULE_LOCATIONS[3]);

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.5;
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0;

        public static final double XY_kP = 8;
        public static final double XY_kI = 0;
        public static final double XY_kD = 0.05;

        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                4,
                4);
        public static final TrapezoidProfile.Constraints XY_TALL_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.5,
                2.5);
        public static final TrapezoidProfile.Constraints AUTO_FAST_CONSTRAINTS = new TrapezoidProfile.Constraints(
                4.0,
                4.5);
        public static final TrapezoidProfile.Constraints AUTO_TALL_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.5,
                2.5);
        public static final TrapezoidProfile.Constraints AUTO_STANDARD_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.2,
                1.7);

        // lifted from 6328's DriveToPose
        public static final double XY_FF_MIN_RANGE = 0.1;
        public static final double XY_FF_MAX_RANGE = 0.15;

        public static final double THETA_kP = 8;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.1;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                5,
                12);
        public static final TrapezoidProfile.Constraints THETA_SLOW_CONSTRAINTS = new TrapezoidProfile.Constraints(
                5,
                3);

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                MASS_KG, MOI,
                new ModuleConfig(
                        WHEEL_RADIUS_METERS,
                        SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                        WHEEL_COF, SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                        SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT, 1),
                SWERVE_MODULE_LOCATIONS);
    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
            ELEVATOR_RIGHT(0, 53, false),
            ELEVATOR_L1_RIGHT(0, 13, false),
            ELEVATOR_L2_RIGHT(14, 26, false),
            ELEVATOR_L3_RIGHT(27, 40, false),
            ELEVATOR_L4_RIGHT(41, 53, false),
            ELEVATOR_CENTER(54, 94, false),
            ELEVATOR_CENTER_RIGHT(54, 74, false),
            ELEVATOR_CENTER_LEFT(75, 94, false),
            ELEVATOR_LEFT(95, 132, true),
            ELEVATOR_L1_LEFT(124, 132, true),
            ELEVATOR_L2_LEFT(114, 123, true),
            ELEVATOR_L3_LEFT(105, 113, true),
            ELEVATOR_L4_LEFT(95, 104, true),
            ALL(0, 132, true);

            private final int startIdx;
            private final int endIdx;
            private final boolean inverted;

            private LEDSection(int startIdx, int endIdx, boolean inverted) {
                this.startIdx = startIdx;
                this.endIdx = endIdx;
                this.inverted = inverted;
            }

            private LEDSection(int startIdx, int endIdx) {
                this(startIdx, endIdx, false);
            }

            @Override
            public int start() {
                return startIdx;
            }

            @Override
            public int end() {
                return endIdx;
            }

            @Override
            public boolean inverted() {
                return inverted;
            }

            @Override
            public int length() {
                return endIdx - startIdx + 1;
            }
        }

        public static final int PORT = 6;
        public static final int LENGTH = 133;

        public static final List<LEDState> DEFAULT_STATES = List.of(LEDState.FIRE);
    }

    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_TELEOP_STD_DEVS = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);
        // TODO
        public static final Matrix<N3, N1> SINGLE_TAG_PRECISE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Double.MAX_VALUE);

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            CAMERA_CONSTANTS.WIDTH = 1600;
            CAMERA_CONSTANTS.HEIGHT = 1304;
            CAMERA_CONSTANTS.FOV = 95.39;
            CAMERA_CONSTANTS.FPS = 35;
            CAMERA_CONSTANTS.AVG_LATENCY = 30;
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
        }

        // TODO simvalue
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                // left camera
                new Transform3d(
                        new Translation3d(
                                0.29845, // 0.29845 m
                                0.1905, // 0.1905 m
                                0.210531964),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(-35))),
                // right camera
                new Transform3d(
                        new Translation3d(
                                0.29845,
                                -0.1905,
                                0.210531964),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(35))),
        };
    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 3.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.1;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_LIMIT = 0.8;
        public static final double AUTO_DRIVE_ADAPTIVE_SCALE_FACTOR = 0.5;
    }
}
