package frc.robot.Subsystem.Configs;

public class SwerveModuleConfig {
    private String name;
    private double wheelDiameter;
    private double lowGearRatio;
    private double highGearRatio;
    private double lowGearMaxSpeed;
    private double highGearMaxSpeed;

    private double angleP;
    private double angleI;
    private double angleD;
    private double angleFF;

    private double speedP;
    private double speedI;
    private double speedD;

    private double speedS;
    private double speedV;

    public String getName() {
        return this.name;
    }

    public double getLowGearMaxSpeed() {
        return this.lowGearMaxSpeed;
    }

    public double getHighGearMaxSpeed() {
        return this.highGearMaxSpeed;
    }

    public double getAngleFF() {
        return this.angleFF;
    }

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

    public double getAngleP() {
        return this.angleP;
    }

    public double getAngleI() {
        return this.angleI;
    }

    public double getAngleD() {
        return this.angleD;
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
}
