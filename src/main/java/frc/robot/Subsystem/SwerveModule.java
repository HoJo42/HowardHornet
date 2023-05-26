package frc.robot.Subsystem;

import java.lang.annotation.Native;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
    
    private WPI_TalonSRX speedMotor;
    private ProfiledPIDController speedPID;

    private CANSparkMax rotationMotor;
    private AnalogEncoder rotationEncoder;
    private ProfiledPIDController rotationPID;
    
    private final double kRotationCountsPerRevolution;

    public SwerveModule(WPI_TalonSRX speedMotor,
                        ProfiledPIDController speedPID,
                        CANSparkMax rotationMotor,
                        AnalogEncoder rotationEncoder,
                        ProfiledPIDController rotationPID,
                        double angleCountsPerRevolution){
        
        this.speedMotor = speedMotor;
        this.speedPID = speedPID;
        this.rotationMotor = rotationMotor;
        this.rotationEncoder = rotationEncoder;
        this.rotationPID = rotationPID;
        this.kRotationCountsPerRevolution = angleCountsPerRevolution;
    }

    public void setModule(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getRotation2d());

        double rotationSetpointnative = radiansToNative(state.angle.getRadians());
        double rotationPIDOutput = rotationPID.calculate(getAngle(), rotationSetpointnative);

        if (state.speedMetersPerSecond == 0.0)
            rotationMotor.setVoltage(0);
        else
            rotationMotor.setVoltage(rotationPIDOutput);

        
    }

    private double radiansToNative(double radians) {
        return radians / (2 * Math.PI) * kRotationCountsPerRevolution;
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(nativeToRadians(getAngle()));
    }

    /** 
   * Converts native absolute encoder counts to radians
   * @param encoderCounts Absolute encoder counts
   * @return Angle of the module in radians
   */
  public double nativeToRadians(double encoderCounts) {
    return encoderCounts * (2 * Math.PI) / kRotationCountsPerRevolution;
  }

  public double getAngle() {
    return rotationEncoder.getAbsolutePosition();
  }
}
