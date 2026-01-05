package frc.robot;

import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876); // TODO 2025 value
    public static final double fieldWidth = Units.inchesToMeters(317); // TODO 2025 value

    public static Pose2d rotateBluePoseIfNecessary(Pose2d original) {
        return Utils.shouldFlipValueToRed()
                ? Reflector.rotatePoseAcrossField(original, FieldConstants.fieldLength, FieldConstants.fieldWidth)
                : original;
    }
}