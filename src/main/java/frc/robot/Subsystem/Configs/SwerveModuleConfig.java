package frc.robot.Subsystem.Configs;

public class SwerveModuleConfig {
    private double angleCountsPerRevolution;

    private double speedP;
    private double speedI;
    private double speedD;
    private double speedMaxVelocity;
    private double speedMaxAcceleration;

    private double angleP;
    private double angleI;
    private double angleD;
    private double angleMaxVelocity;
    private double angleMaxAcceleration;

    private double angleS;
    private double angleV;

    public double getAngleS() {
        return this.angleS;
    }

    public void setAngleS(double angleS) {
        this.angleS = angleS;
    }

    public double getAngleV() {
        return this.angleV;
    }

    public void setAngleV(double angleV) {
        this.angleV = angleV;
    }

    public double getAngleP() {
        return this.angleP;
    }

    public void setAngleP(double angleP) {
        this.angleP = angleP;
    }

    public double getAngleI() {
        return this.angleI;
    }

    public void setAngleI(double angleI) {
        this.angleI = angleI;
    }

    public double getAngleD() {
        return this.angleD;
    }

    public void setAngleD(double angleD) {
        this.angleD = angleD;
    }

    public double getAngleMaxVelocity() {
        return this.angleMaxVelocity;
    }

    public void setAngleMaxVelocity(double angleMaxVelocity) {
        this.angleMaxVelocity = angleMaxVelocity;
    }

    public double getAngleMaxAcceleration() {
        return this.angleMaxAcceleration;
    }

    public void setAngleMaxAcceleration(double angleMaxAcceleration) {
        this.angleMaxAcceleration = angleMaxAcceleration;
    }

    public double getSpeedP() {
        return this.speedP;
    }

    public void setSpeedP(double speedP) {
        this.speedP = speedP;
    }

    public double getSpeedI() {
        return this.speedI;
    }

    public void setSpeedI(double speedI) {
        this.speedI = speedI;
    }

    public double getSpeedD() {
        return this.speedD;
    }

    public void setSpeedD(double speedD) {
        this.speedD = speedD;
    }

    public double getSpeedMaxVelocity() {
        return this.speedMaxVelocity;
    }

    public void setSpeedMaxVelocity(double speedMaxVelocity) {
        this.speedMaxVelocity = speedMaxVelocity;
    }

    public double getSpeedMaxAcceleration() {
        return this.speedMaxAcceleration;
    }

    public void setSpeedMaxAcceleration(double speedMaxAcceleration) {
        this.speedMaxAcceleration = speedMaxAcceleration;
    }

    public double getAngleCountsPerRevolution() {
        return angleCountsPerRevolution;
    }
    
}
