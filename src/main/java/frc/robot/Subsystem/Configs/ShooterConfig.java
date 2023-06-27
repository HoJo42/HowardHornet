package frc.robot.Subsystem.Configs;

public class ShooterConfig {
    private double defaultRPM;
    
    private double ks;
    private double kv;

    private double kp;
    private double ki;
    private double kd;

    public double getDefaultRPM() {
        return defaultRPM;
    }

    public double getKS() {
        return ks;
    }

    public double getKV() {
        return kv;
    }

    public double getKP() {
        return kp;
    }

    public double getKI() {
        return ki;
    }

    public double getKD() {
        return kd;
    }
}
