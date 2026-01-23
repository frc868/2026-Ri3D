package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import java.util.List;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlog.loggers.MetadataLogger.MetadataRecord;
import com.techhounds.houndutil.houndlog.FaultLogger;
import com.techhounds.houndutil.houndlog.LogAnnotationHandler;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.LogProfiles;
import com.techhounds.houndutil.houndlog.LogType;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.loggers.MetadataLogger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.LEDs.LEDState;

public class Constants {
    public static final double LOOP_TIME = 0.020; // 20ms

    // public static final MetadataRecord BUILD_METADATA = new MetadataRecord(
    //         BuildConstants.MAVEN_NAME, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE,
    //         BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);

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

        public static final int FRONT_LEFT_STEER_ENCODER_ID = 1;
        public static final int FRONT_RIGHT_STEER_ENCODER_ID = 2;
        public static final int BACK_LEFT_STEER_ENCODER_ID = 3;
        public static final int BACK_RIGHT_STEER_ENCODER_ID = 4;

        public static final int PIGEON_ID = 0;

        public static final boolean DRIVE_MOTORS_INVERTED = true;
        public static final boolean STEER_MOTORS_INVERTED = false;
        public static final boolean STEER_CANCODERS_INVERTED = RobotBase.isReal() ? false : false;

        public static final Angle FRONT_LEFT_OFFSET = Rotations.of(0.37451171875);
        public static final Angle FRONT_RIGHT_OFFSET = Rotations.of(-0.180556640625);
        public static final Angle BACK_LEFT_OFFSET = Rotations.of(-0.222470703);
        public static final Angle BACK_RIGHT_OFFSET = Rotations.of(-0.311650390625);

        /** Distance between left and right wheels. */
        public static final Distance FRAME_SIDE_LENGTH = Inches.of(27.5);
        public static final Distance TRACK_WIDTH_METERS = FRAME_SIDE_LENGTH.minus(Inches.of(5));
        /** Distance between front and back wheels. */
        public static final Distance WHEEL_BASE_METERS = FRAME_SIDE_LENGTH.minus(Inches.of(5));
        /** Distance between the center of the robot and a module. */
        public static final Distance DRIVE_BASE_RADIUS_METERS = Meters
                .of(Math.sqrt(
                        Math.pow(TRACK_WIDTH_METERS.in(Meters), 2) +
                                Math.pow(TRACK_WIDTH_METERS.in(Meters), 2))
                        / 2.0);
        public static final Mass MASS_KG = Pounds.of(135); // TODO simvalue
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(6.0);
        public static final Distance WHEEL_RADIUS_METERS = Meters.of(0.04772270733456543);
        public static final double WHEEL_COF = 1.3;

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 0;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.12246;
            SWERVE_CONSTANTS.DRIVE_kV = 0.60644;
            SWERVE_CONSTANTS.DRIVE_kA = 0.057917;
            SWERVE_CONSTANTS.STEER_kP = 100.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 1.0;
            SWERVE_CONSTANTS.STEER_kS = 0;
            SWERVE_CONSTANTS.STEER_kV = 0;
            SWERVE_CONSTANTS.STEER_kA = 0;

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.14;
            SWERVE_CONSTANTS.STEER_GEARING = 24.0 / 1.0;
            SWERVE_CONSTANTS.COUPLING_RATIO = 50.0 / 13.0; // Thrifty Narrow
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS_METERS.in(Meters);
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5.41;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 32;
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 125.5 * 2 * Math.PI;
            // max velocity in 1/10 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 10 * 125.5 * 2 * Math.PI;

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 85;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 30;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = Utils.getKrakenX44(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.01;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 14.51;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 60;

        public static final Translation2d[] SWERVE_MODULE_LOCATIONS = new Translation2d[] {
                new Translation2d(WHEEL_BASE_METERS.in(Meters) / 2, TRACK_WIDTH_METERS.in(Meters) / 2),
                new Translation2d(WHEEL_BASE_METERS.in(Meters) / 2, -TRACK_WIDTH_METERS.in(Meters) / 2),
                new Translation2d(-WHEEL_BASE_METERS.in(Meters) / 2, TRACK_WIDTH_METERS.in(Meters) / 2),
                new Translation2d(-WHEEL_BASE_METERS.in(Meters) / 2, -TRACK_WIDTH_METERS.in(Meters) / 2) };

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

        // from 6328's DriveToPose
        public static final double XY_FF_MIN_RANGE = 0.1;
        public static final double XY_FF_MAX_RANGE = 0.15;

        public static final double THETA_kP = 8;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.5;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                10,
                15);

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                MASS_KG.in(Kilograms), MOI.in(KilogramSquareMeters),
                new ModuleConfig(
                        WHEEL_RADIUS_METERS.in(Meters),
                        SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                        WHEEL_COF, SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                        SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT, 1),
                SWERVE_MODULE_LOCATIONS);
    }

    public static final class Shooter {
        public static final int PRIMARY_MOTOR_ID = 9;
        public static final int SECONDARY_MOTOR_ID = 10;
        public static final int TERTIARY_MOTOR_ID = 11;
        public static final int QUATERNARY_MOTOR_ID = 12;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60Foc(2);
        public static final double GEARING = 24.0 / 18.0; // TODO
        public static final double WHEEL_AXLE_MASS = Units.lbsToKilograms(8); // TODO
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5); // TODO
        // 2.5lb, 2in radius, 1/2mr^2
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = (1.0 / 2.0) * WHEEL_AXLE_MASS
                * Math.pow(WHEEL_RADIUS, 2);
        public static final int CURRENT_LIMIT = 100; // TODO

        public static final double TOLERANCE = 0;

        // TODO
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.19353;
        public static final double kV = 0.1669;
        public static final double kA = 0.014113;

        public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 50; // TODO
        public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 50; // TODO

        // TODO

        public static final double GOAL_POSITION_ITERATIONS = 5;
        public static final double ACCELERATION_COMPENSATION_FACTOR = 0.0;

        public static final Transform3d BALL_TRANSFORM_LEFT = new Transform3d(-0.24, 0.09, 0.5, Rotation3d.kZero);
        public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);
        public static final Transform3d BALL_TRANSFORM_RIGHT = new Transform3d(-0.24, -0.09, 0.5, Rotation3d.kZero);

        public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_SPEED = new InterpolatingDoubleTreeMap();
        static {
            DISTANCE_TO_SHOT_SPEED.put(2.07, 7.0);
            // DISTANCE_TO_SHOT_SPEED.put(2.41, 41.0);
            // DISTANCE_TO_SHOT_SPEED.put(3.20, 45.0);
            // DISTANCE_TO_SHOT_SPEED.put(3.87, 49.0);
            // DISTANCE_TO_SHOT_SPEED.put(4.57, 52.0);
            DISTANCE_TO_SHOT_SPEED.put(4.92, 9.0);
            // DISTANCE_TO_SHOT_SPEED.put(0.0, 7.0);
            // DISTANCE_TO_SHOT_SPEED.put(5.0, 8.25);
            // DISTANCE_TO_SHOT_SPEED.put(10.0, 10.0);
        }

        public static final InterpolatingDoubleTreeMap SHOT_SPEED_TO_RPS = new InterpolatingDoubleTreeMap();
        static {

        }

        // temporary easy solution
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPS = new InterpolatingDoubleTreeMap();
        static {
            DISTANCE_TO_RPS.put(2.07, 38.0);
            DISTANCE_TO_RPS.put(2.41, 41.0);
            DISTANCE_TO_RPS.put(3.20, 45.0);
            DISTANCE_TO_RPS.put(3.87, 49.0);
            DISTANCE_TO_RPS.put(4.57, 52.0);
            DISTANCE_TO_RPS.put(4.92, 58.0);
        }
    }

    public static final class ShooterHood {
        public static enum HoodPosition {
            TOP(1.410),
            MIDDLE(2.25),
            BOTTOM(2.878);

            public final double value;

            private HoodPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 16;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60Foc(1);
        public static final double GEARING = 40;
        public static final double LENGTH_METERS = 0.2032;
        public static final double MASS_KG = 0.90718474;
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = SingleJointedArmSim.estimateMOI(
                LENGTH_METERS,
                MASS_KG);

        public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 5;
        public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 5;

        public static final double MIN_ANGLE_RADIANS = 1.410;
        public static final double MAX_ANGLE_RADIANS = 2.878;

        public static final double CURRENT_LIMIT = 100;

        public static final double TOLERANCE = 0.01;

        public static final double kP = 100.0;
        public static final double kI = 0.0;
        public static final double kD = 1.0;
        public static final double kS = RobotBase.isReal() ? 0.1 / 5.0 : 0.0;
        public static final double kG = 0.28 / 5.0;
        public static final double kV = 4.98;
        public static final double kA = 0.01 / 5.0;

        public static final InterpolatingDoubleTreeMap SHOT_ANGLE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
        static {
            SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(59), 2.878);
            SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(45), 2.7);
            SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(40), 2.6);
            SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(38), 2.5);
            SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(37), 2.4);
            SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(30), 2.3);
        }

        // temporary easy solution
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
        static {
            DISTANCE_TO_HOOD_ANGLE.put(2.07, 2.878);
            DISTANCE_TO_HOOD_ANGLE.put(2.41, 2.853);
            DISTANCE_TO_HOOD_ANGLE.put(3.20, 2.764);
            DISTANCE_TO_HOOD_ANGLE.put(3.87, 2.764);
            DISTANCE_TO_HOOD_ANGLE.put(4.57, 2.764);
            DISTANCE_TO_HOOD_ANGLE.put(4.92, 2.694);
        }

        public static double getHoodAngle(double targetPitch) {
            return targetPitch + Math.PI / 2.0;
        }
    }

    public static final class Intake {
        public static enum IntakePosition {
            BOTTOM(Radians.of(-0.3)),
            GROUND(Radians.of(-0.15)),
            JOG(Radians.of(0.5)),
            MINISTOW(Radians.of(1.12469017)),
            STOW(Radians.of(1.58));

            public final Angle value;

            private IntakePosition(Angle value) {
                this.value = value;
            }
        }

        public static final int LEFT_ARM_MOTOR_ID = 13;
        public static final int RIGHT_ARM_MOTOR_ID = 14;
        public static final int ROLLER_MOTOR_ID = 15;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(2);
        public static final double GEARING = 23.0 * 1.2587890624; // TODO
        public static final double LENGTH_METERS = 0.26035; // TODO
        public static final double MASS_KG = 6; // TODO
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = SingleJointedArmSim.estimateMOI(
                LENGTH_METERS,
                MASS_KG);

        public static final double MIN_ANGLE_RADIANS = -0.3; // TODO
        public static final double MAX_ANGLE_RADIANS = 1.58; // TODO

        public static final double ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / GEARING; // TODO
        public static final int ARM_CURRENT_LIMIT = 30; // TODO
        public static final int ROLLER_CURRENT_LIMIT = 75; // TODO

        // TODO
        public static final double kP = 40;
        public static final double kI = 0.0;
        public static final double kD = 1;
        public static final double kS = RobotBase.isReal() ? 0.07 : 0.0;
        public static final double kG = 0.21;
        public static final double kV = 3.60;
        public static final double kA = 0.05;
        public static final double TOLERANCE = 0.05;

        public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 30;

        // TODO
        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(-0.19, 0, 0.299,
                new Rotation3d(0, -Units.degreesToRadians(35), Math.PI));
    }

    public static final class Climber {

        public static final int TOP_MOTOR_ID = 17;
        public static final int BOTTOM_MOTOR_ID = 18;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 60;
    }

    public static final class Hopper {
        public static final int MOTOR_ID = 19;
        public static final DCMotor MOTOR_GEARBOX_REPR = Utils.getKrakenX44(1);
        public static final double CURRENT_LIMIT = 80;
    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
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

        public static final List<LEDState> DEFAULT_STATES = List.of(LEDState.INITIALIZED_CONFIRM);
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

        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                // left camera
                new Transform3d(
                        new Translation3d(
                                Inches.of(11.724).in(Meters),
                                Inches.of(11.029).in(Meters),
                                Inches.of(8.290).in(Meters)),
                        new Rotation3d(0, Units.degreesToRadians(-15),
                                Units.degreesToRadians(-25))),
                // right camera
                new Transform3d(
                        new Translation3d(
                                Inches.of(-11).in(Meters),
                                Inches.of(-12).in(Meters),
                                Inches.of(21).in(Meters)),
                        new Rotation3d(0, Units.degreesToRadians(-12),
                                Units.degreesToRadians(15))),
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
