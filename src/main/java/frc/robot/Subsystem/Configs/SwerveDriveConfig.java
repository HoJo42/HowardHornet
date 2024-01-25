package frc.robot.Subsystem.Configs;

import SOTAlib.Math.Conversions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveDriveConfig {
    private double wheelBase;
    private double trackWidth;

    /**
     * @return robot's track width in meters
     */
    public double getWheelBase() {
        return this.wheelBase * Conversions.METERS_PER_INCH;
    }

    /**
     * @return robot's track width in Meters
     */
    public double getTrackWidth() {
        return this.trackWidth * Conversions.METERS_PER_INCH;
    }

    /**
     * Order Front right, Front left, Back Left, Back Right
     */
    public Translation2d[] generateModuleTranslations() {
        Translation2d[] moduleTranslations = {
            new Translation2d(getWheelBase() / 2, -getTrackWidth() / 2),
            new Translation2d(getWheelBase() / 2, getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, getTrackWidth() / 2), 
            new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2)
        };
        return moduleTranslations;
        
    }

    /*
     * Generates it's own module translations so that nothing else can disturb them
     */
    public SwerveDriveKinematics generateKinematics() {
        return new SwerveDriveKinematics(generateModuleTranslations());
    }

    public SwerveDriveOdometry generateOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        return new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions);
    }
}
