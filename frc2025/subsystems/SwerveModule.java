package team10034.frc2025.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final SparkMax steerMotor;
    private final SparkMax driveMotor;
    private final CANcoder canCoder;
    private final RelativeEncoder driveEncoder;
    private final PIDController steerPIDController;
    private final PIDController drivePIDController;

    private final double offset;
    private final double tolerance = 1;

    public SwerveModule(int steerMotorID, int canCoderID, double offset, int driveMotorID) {
        this.steerMotor = new SparkMax(steerMotorID, SparkMax.MotorType.kBrushless);
        this.driveMotor = new SparkMax(driveMotorID, SparkMax.MotorType.kBrushless);
        this.canCoder = new CANcoder(canCoderID);
        this.driveEncoder = driveMotor.getEncoder();

        this.offset = offset;
        this.steerPIDController = new PIDController(0.0048, 0.0, 0.0);
        this.drivePIDController = new PIDController(0.000068, 0.00042, 0.0);

        SmartDashboard.putNumber("Steer Kp", 0.0048);
        SmartDashboard.putNumber("Steer Ki", 0.0);
        SmartDashboard.putNumber("Steer Kd", 0.0);
        SmartDashboard.putNumber("Drive P", 0.000068);
        SmartDashboard.putNumber("Drive I", 0.00042);
        SmartDashboard.putNumber("Drive D", 0.0);
    }

    public double getAbsoluteAngle() {
        return positionToDegrees(canCoder.getPosition().getValueAsDouble(), offset);
    }

    public void setTargetAngle(double targetAngle) {
        double Kp = SmartDashboard.getNumber("Steer Kp", 0.0048);
        double Ki = SmartDashboard.getNumber("Steer Ki", 0.0);
        double Kd = SmartDashboard.getNumber("Steer Kd", 0.0);

        steerPIDController.setP(Kp);
        steerPIDController.setI(Ki);
        steerPIDController.setD(Kd);

        double currentAngle = getAbsoluteAngle();
        double angleError = targetAngle - currentAngle;

        angleError = ((angleError + 180) % 360 + 360) % 360 - 180;

        if (Math.abs(angleError) < tolerance) {
            steerMotor.set(0.0);
            return;
        }

        double pidOutput = steerPIDController.calculate(angleError);
        steerMotor.set(pidOutput);
    }

    public void setTargetSpeed(double targetVelocity) {
        double driveP = SmartDashboard.getNumber("Drive P", 0.000068);
        double driveI = SmartDashboard.getNumber("Drive I", 0.00042);
        double driveD = SmartDashboard.getNumber("Drive D", 0.0);

        drivePIDController.setP(driveP);
        drivePIDController.setI(driveI);
        drivePIDController.setD(driveD);

        double driveOutput = MathUtil.clamp(
            drivePIDController.calculate(driveEncoder.getVelocity(), targetVelocity),
            -1.0, 1.0
        );
        driveMotor.set(driveOutput);
    }

    private double positionToDegrees(double position, double offsetAngle) {
        double angle = (position * 360.0) - offsetAngle;
        return ((angle % 360) + 360) % 360;
    }

    public double getSpeed(){
        return driveEncoder.getVelocity();
    }
}