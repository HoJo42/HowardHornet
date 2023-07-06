package frc.robot.Subsystem;

import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.Pneumatics.DoubleSolenoidShifter;
import SOTAlib.Pneumatics.GearShifter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.SwerveDriveConfig;

public class SwerveDrive extends SubsystemBase {
  private SwerveDriveKinematics mKinematics;
  private SwerveModule[] modules;
  private SOTA_Gyro gyro;
  private GearShifter shifter;
  private boolean fieldCentric;


  public SwerveDrive(SwerveModule[] modules, SOTA_Gyro gyro, GearShifter shifter, SwerveDriveConfig config) {
    this.modules = modules;
    this.gyro = gyro;
    this.shifter = shifter;
    shifter.shift(0);
    for (SwerveModule loopModule : modules) {
      loopModule.setCurrentGear(0);
    }
    this.fieldCentric = true;
    this.mKinematics = config.generateKinematics();
  }

  public void drive (double frwrd, double strf, double rttn) {
    frwrd = MathUtil.clamp(frwrd, -1, 1) * getLowestMaxSpeed();
    strf = MathUtil.clamp(strf, -1, 1) * getLowestMaxSpeed();
    rttn = MathUtil.clamp(rttn, -1, 1) * getLowestMaxAngularVelocity();
  }

  /**
   * 
   * @param speeds Raw inputs in a ChassisSpeeds object
   */
  public void drive(ChassisSpeeds speeds) {
    if (fieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
    }

    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, getLowestMaxSpeed());

    for (int i = 0; i < moduleStates.length; i++) {
      modules[i].setModule(moduleStates[i]);
    }
  }

  public void toggleFieldCentric() {
    if (fieldCentric) {
      fieldCentric = false;
    }else {
      fieldCentric = true;
    }
  }

  public double getLowestMaxSpeed() {
    double output = modules[0].getCurrentMaxSpeed();
    for (SwerveModule loopModule : modules) {
      if (loopModule.getCurrentMaxSpeed() < output) {output = loopModule.getCurrentMaxSpeed();};
    }
    return output;
  }

  public double getLowestMaxAngularVelocity() {
    double output = modules[0].getMaxAngularVelocity();
    for (SwerveModule loopModule : modules) {
      if (loopModule.getMaxAngularVelocity() < output) {output = loopModule.getMaxAngularVelocity();};
    }
    return output;
  }

  public void shift(int gear) {
    shifter.shift(gear);
    for (SwerveModule loopModule : modules) {
      loopModule.setCurrentGear(gear);
    }
  }
}