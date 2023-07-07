package frc.robot.Subsystem.Configs;

public class SwerveModuleConfig {
    // private double angleCountsPerRevolution;
    // private double speedCountsPerRevolution;
    private double wheelDiameter;
    private double lowGearRatio;
    private double highGearRatio;
    private double lowGearMaxSpeed;
    private double highGearMaxSpeed;

    private double angleP;
    private double angleI;
    private double angleD;
    private double angleMaxVelocity;
    private double angleMaxAcceleration;

    private double angleS;
    private double angleV;

    private double speedP;
    private double speedI;
    private double speedD;
    private double speedMaxVelocity;
    private double speedMaxAcceleration;

    private double speedS;
    private double speedV;

    public double getLowGearMaxSpeed() {
        return this.lowGearMaxSpeed;
    }

    public double getHighGearMaxSpeed() {
        return this.highGearMaxSpeed;
    }

    // public double getSpeedCountsPerRevolution() {
    //     return this.speedCountsPerRevolution;
    // }

    public double getWheelDiameter() {
        return this.wheelDiameter;
    }

    public double getWheelCircumference() {
        return wheelDiameter * Math.PI;
    }

    public double getLowGearRatio() {
        return this.lowGearRatio;
    }

    public double getHighGearRatio() {
        return this.highGearRatio;
    }

    public double getSpeedS() {
        return this.speedS;
    }

    public double getSpeedV() {
        return this.speedV;
    }

    public double getAngleS() {
        return this.angleS;
    }

    public double getAngleV() {
        return this.angleV;
    }

    public double getAngleP() {
        return this.angleP;
    }

    public double getAngleI() {
        return this.angleI;
    }

    public double getAngleD() {
        return this.angleD;
    }

    public double getAngleMaxVelocity() {
        return this.angleMaxVelocity;
    }

    public double getAngleMaxAcceleration() {
        return this.angleMaxAcceleration;
    }

    public double getSpeedP() {
        return this.speedP;
    }

    public double getSpeedI() {
        return this.speedI;
    }

    public double getSpeedD() {
        return this.speedD;
    }

    public double getSpeedMaxVelocity() {
        return this.speedMaxVelocity;
    }

    public double getSpeedMaxAcceleration() {
        return this.speedMaxAcceleration;
    }

    // public double getAngleCountsPerRevolution() {
    //     return angleCountsPerRevolution;
    // }

}
