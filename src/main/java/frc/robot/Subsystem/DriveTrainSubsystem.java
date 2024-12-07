package frc.robot.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//mport frc.robot.Subsystem.Encoder.EncoderMovement;

public class DriveTrainSubsystem extends SubsystemBase {
/* added Nov 22 from https://www.youtube.com/watch?v=gmx0NKyH5nI*/
    private final CANSparkMax leftFrontMotor;
    private final CANSparkMax rightFrontMotor;
    private final CANSparkMax leftBackMotor; 
    private final CANSparkMax rightBackMotor; 
    private final CANSparkMax leftFrontTurn;
    private final CANSparkMax leftBackTurn;
    private final CANSparkMax rightFrontTurn;
    private final CANSparkMax rightBackTurn;

    private final RelativeEncoder leftFrontEncoder;
    private final RelativeEncoder rightFrontEncoder;
    private final RelativeEncoder leftBackEncoder; 
    private final RelativeEncoder rightBackEncoder;
    private final RelativeEncoder leftFrontTurnEncoder;
    private final RelativeEncoder rightFrontTurnEncoder;
    private final RelativeEncoder leftBackTurnEncoder; 
    private final RelativeEncoder rightBackTurnEncoder;

    private double m_P = 0.00006;     // factor for "proportional" control
    private double m_I = 0.0;     // factor for "integral" control
    private double m_D = 0.00;     // factor for "derivative" control
    private double m_F = 0;                 // factor for feedforward term
    private double m_setpoint = 0.0;
    private final double encoderResolution = 4096;

    public DriveTrainSubsystem(int leftStickPort, int rightStickPort,
    int leftFrontMotorID, int leftBackMotorID, int rightFrontMotorID, int rightBackMotorID,
    int leftFrontTurnID,  int leftBackTurnID,  int rightFrontTurnID, int rightBackTurnID) {

        leftFrontMotor  = new CANSparkMax(leftFrontMotorID, CANSparkMax.MotorType.kBrushless);
        rightFrontMotor = new CANSparkMax(rightFrontMotorID, CANSparkMax.MotorType.kBrushless);
        leftBackMotor   = new CANSparkMax(leftBackMotorID, CANSparkMax.MotorType.kBrushless);
        rightBackMotor  = new CANSparkMax(rightBackMotorID, CANSparkMax.MotorType.kBrushless);

        leftFrontTurn   = new CANSparkMax(leftFrontTurnID, CANSparkMax.MotorType.kBrushless);
        leftBackTurn    = new CANSparkMax(leftBackTurnID, CANSparkMax.MotorType.kBrushless);
        rightFrontTurn  = new CANSparkMax(rightFrontTurnID, CANSparkMax.MotorType.kBrushless);
        rightBackTurn   = new CANSparkMax(rightBackTurnID, CANSparkMax.MotorType.kBrushless);

        leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftFrontTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftBackTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);

        configurePID(leftFrontMotor);
        configurePID(rightFrontMotor);
        configurePID(leftBackMotor);
        configurePID(rightBackMotor);
        configurePID(leftFrontTurn);
        configurePID(leftBackTurn);
        configurePID(rightFrontTurn);
        configurePID(rightBackTurn);

        leftFrontEncoder  = leftFrontMotor.getEncoder();
        rightFrontEncoder = rightFrontMotor.getEncoder();
        leftBackEncoder   = leftBackMotor.getEncoder();
        rightBackEncoder  = rightBackMotor.getEncoder();

        leftFrontTurnEncoder  = leftFrontTurn.getEncoder();
        rightFrontTurnEncoder = rightFrontTurn.getEncoder();
        leftBackTurnEncoder   = leftBackTurn.getEncoder();
        rightBackTurnEncoder  = rightBackTurn.getEncoder();
    }
    public double toDegrees(double encoderCounts) {
      return (encoderCounts / encoderResolution) * 360;
  }
    public void arcadeDrive(double speed, double rotation) {
        double leftMotorOutput;
        double rightMotorOutput;
        
        leftMotorOutput  = speed + rotation; 
        rightMotorOutput = speed - rotation;

        leftFrontMotor.set(leftMotorOutput);
        rightFrontMotor.set(rightMotorOutput);
        leftBackMotor.set(leftMotorOutput); 
        rightBackMotor.set(rightMotorOutput);

        leftFrontTurn.set(leftMotorOutput);
        rightFrontTurn.set(rightMotorOutput);
        leftBackTurn.set(leftMotorOutput); 
        rightBackTurn.set(rightMotorOutput);
    }

    public void setSetpoint(double setpoint) { // 設定PID目標值
      m_setpoint = setpoint;
      SmartDashboard.putNumber("Setpoint", m_setpoint);
  }

    private void configurePID(CANSparkMax motor) {
      motor.getPIDController().setP(m_P); // 調整這些值以符合你的需求
      motor.getPIDController().setI(m_I);
      motor.getPIDController().setD(m_D);
      motor.getPIDController().setIZone(100);
      motor.getPIDController().setFF(m_F);
      motor.getPIDController().setOutputRange(-1, 1);
   }
    
    public void moveWithPID(double angle, double speed){
    //  double countsPerDegree = encoderCount / 360.0;
    //  double targetCounts = angle * countsPerDegree;
    double countsPerRadian = 4096.0 / (2 * Math.PI); //弧度
    double targetCounts = angle * countsPerRadian; // 角度
    double leftSpeed = speed; // 例如，從搖桿讀取速度
    double rightSpeed = speed;  // 例如，從搖桿讀取速度
    double k = 1;

        leftFrontMotor.getPIDController().setReference(leftSpeed*k, CANSparkMax.ControlType.kVelocity);
        rightFrontMotor.getPIDController().setReference(rightSpeed*k, CANSparkMax.ControlType.kVelocity);
        leftBackMotor.getPIDController().setReference(leftSpeed*k, CANSparkMax.ControlType.kVelocity);
        rightBackMotor.getPIDController().setReference(rightSpeed*k, CANSparkMax.ControlType.kVelocity);

        leftFrontTurn.getPIDController().setReference(targetCounts, CANSparkMax.ControlType.kPosition);
        rightFrontTurn.getPIDController().setReference(targetCounts, CANSparkMax.ControlType.kPosition);
        leftBackTurn.getPIDController().setReference(targetCounts, CANSparkMax.ControlType.kPosition);
        rightBackTurn.getPIDController().setReference(targetCounts, CANSparkMax.ControlType.kPosition);

     //   System.out.print("Turn_ POSITION");
    //    System.out.print(String.format("%.2f ", toDegrees(leftFrontTurnEncoder.getPosition())));
    //    System.out.print(String.format("%.2f ", toDegrees(leftBackTurnEncoder.getPosition())));
      //  System.out.print(String.format("%.2f ", toDegrees(rightFrontTurnEncoder.getPosition())));
     //   System.out.println(String.format("%.2f ", toDegrees(rightBackTurnEncoder.getPosition())));
       
        System.out.print("Motor VELOCITY");
        System.out.print(String.format("%.2f", leftFrontEncoder.getVelocity()));
        System.out.print(String.format("%.2f", leftBackEncoder.getVelocity()));
        System.out.print(String.format("%.2f", rightFrontEncoder.getVelocity()));
        System.out.println(String.format("%.2f", rightBackEncoder.getVelocity()));
  }
    public void displayEncoderValues(){
        SmartDashboard.putNumber("Left Front Encoder", leftFrontEncoder.getPosition());
        SmartDashboard.putNumber("Left Back Encoder", leftBackEncoder.getPosition()); 
        SmartDashboard.putNumber("Right Front Encoder", rightFrontEncoder.getPosition());
        SmartDashboard.putNumber("Right Back Encoder", rightBackEncoder.getPosition()); 
        SmartDashboard.putNumber("Left Front T Encoder", leftFrontTurnEncoder.getPosition());
        SmartDashboard.putNumber("Left Back T Encoder", leftBackTurnEncoder.getPosition()); 
        SmartDashboard.putNumber("Right Front T Encoder", rightFrontTurnEncoder.getPosition());
        SmartDashboard.putNumber("Right Back T Encoder", rightBackTurnEncoder.getPosition());
    }

    public void getEncoderNumbers(){
        System.out.print("LF:");
        System.out.print(String.format("%.2f", leftFrontEncoder.getPosition()));
        System.out.print("LB:");
        System.out.print(String.format("%.2f", leftBackEncoder.getPosition()));
        System.out.print("  RF:");
        System.out.print(String.format("%.2f", rightFrontEncoder.getPosition()));
        System.out.print("RB:");
        System.out.print(String.format("%.2f", rightBackEncoder.getPosition()));
    }

    @Override
    public void periodic() {
        displayEncoderValues();
    }
}