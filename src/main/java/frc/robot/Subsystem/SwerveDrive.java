package frc.robot.Subsystem;

import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.Pneumatics.DoubleSolenoidShifter;
import SOTAlib.Pneumatics.GearShifter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private SwerveDriveKinematics mKinematics;
  private SwerveModule[] modules;
  private SOTA_Gyro gyro;
  private GearShifter shifter;
  private boolean fieldCentric;

  private double[] maxSpeeds;

  public SwerveDrive(SwerveModule[] modules, SOTA_Gyro gyro, GearShifter shifter) {
    this.modules = modules;
    this.gyro = gyro;
    this.shifter = shifter;
    shifter.shift(0);
    for (SwerveModule loopModule : modules) {
      loopModule.setCurrentGear(0);
    }
    this.fieldCentric = true;
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
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeeds[shifter.getGear()]);

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

}