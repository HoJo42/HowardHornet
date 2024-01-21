package frc.robot.Subsystem;

import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.Math.Conversions;
import SOTAlib.Pneumatics.GearShifter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.SwerveDriveConfig;

public class SwerveDrive extends SubsystemBase {
  private SwerveDriveKinematics mKinematics;
  private SwerveDriveOdometry mOdometry;
  private Pose2d currentPose;
  private SwerveModule[] modules;
  private SOTA_Gyro gyro;
  private GearShifter shifter;
  private double kMaxAngularVelocity = 2 * Math.PI;
  private boolean fieldCentric;
  private ShuffleboardTab sTab;
  private GenericEntry lowestMaxSpeedEntry;

  public SwerveDrive(SwerveModule[] modules, SOTA_Gyro gyro, GearShifter shifter, SwerveDriveConfig config) {
    this.modules = modules;
    this.gyro = gyro;
    this.shifter = shifter;
    shifter.shift(0);
    for (SwerveModule loopModule : modules) {
      loopModule.updateGear();
    }
    this.fieldCentric = true;
    this.mKinematics = config.generateKinematics();
    this.mOdometry = new SwerveDriveOdometry(mKinematics, new Rotation2d(gyro.getAngle()), new SwerveModulePosition[] {
        modules[0].getPosition(), modules[1].getPosition(), modules[2].getPosition(), modules[3].getPosition()
    });

    this.sTab = Shuffleboard.getTab("Swerve");
    this.lowestMaxSpeedEntry = sTab.add("Drive: LowestMaxSpeed", 0.0).getEntry();
  }

  public void drive(double frwrd, double strf, double rttn) {
    frwrd = MathUtil.clamp(frwrd, -1, 1) * Conversions.feetPerSecToMetersPerSec(getLowestMaxSpeed());
    strf = MathUtil.clamp(strf, -1, 1) * Conversions.feetPerSecToMetersPerSec(getLowestMaxSpeed());
    rttn = MathUtil.clamp(rttn, -1, 1) * kMaxAngularVelocity;
    drive(new ChassisSpeeds(frwrd, strf, rttn));
  }

  /**
   * 
   * @param speeds requested speeds in a ChassisSpeeds object
   */
  public void drive(ChassisSpeeds speeds) {
    if (fieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
    }

    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
        Conversions.feetPerSecToMetersPerSec(getLowestMaxSpeed()));

    for (int i = 0; i < moduleStates.length; i++) {
      modules[i].setModule(moduleStates[i]);
    }
  }

  public void autoDrive(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Conversions.feetPerSecToMetersPerSec(getLowestMaxSpeed()));

    for (int i = 0; i < states.length; i++) {
      modules[i].setModule(states[i]);
    }
  }

  public void toggleFieldCentric() {
    if (fieldCentric) {
      fieldCentric = false;
    } else {
      fieldCentric = true;
    }
  }

  /**
   * gets the max speed of the slowest module in the drivetrain
   * 
   * @return lowest speed in FEET PER SECOND
   */
  public double getLowestMaxSpeed() {
    double output = modules[0].getCurrentMaxSpeed();
    for (SwerveModule loopModule : modules) {
      if (loopModule.getCurrentMaxSpeed() < output) {
        output = loopModule.getCurrentMaxSpeed();
      }
    }
    return output;
  }

  private void shift(int gear) {
    shifter.shift(gear);
    for (SwerveModule loopModule : modules) {
      loopModule.updateGear();
    }
  }

  public void shiftUp() {
    shift(1);
  }

  public void shiftDown() {
    shift(0);
  }

  public void resetGyro() {
    gyro.resetAngle();
    updatePose();
  }

  public void updatePose() {
    Rotation2d gyroAngle = new Rotation2d(gyro.getAngle());

    currentPose = mOdometry.update(gyroAngle, new SwerveModulePosition[] {
        modules[0].getPosition(), modules[1].getPosition(), modules[2].getPosition(), modules[3].getPosition()
    });

  }

  public void resetPose(Pose2d newPose) {
    mOdometry.resetPosition(new Rotation2d(gyro.getAngle()), new SwerveModulePosition[] {
        modules[0].getPosition(), modules[1].getPosition(), modules[2].getPosition(), modules[3].getPosition()
    }, newPose);
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return mKinematics;
  }

  @Override
  public void periodic() {
    updatePose();
    lowestMaxSpeedEntry.setDouble(Conversions.feetPerSecToMetersPerSec(getLowestMaxSpeed()));
  }
}