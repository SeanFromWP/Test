package frc.robot;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    private PWM motor1;
    private PWM motor2;
    private PWM motor3;
    private PWM motor4;

    public Robot() {
        motor1 = new PWM(1);
        motor2 = new PWM(2);
        motor3 = new PWM(3);
        motor4 = new PWM(4);
    }

    @Override
    public void teleopPeriodic() {
        double spd1 = SmartDashboard.getNumber("Motor1", 0);
        double spd2 = SmartDashboard.getNumber("Motor2", 0);
        double spd3 = SmartDashboard.getNumber("Motor3", 0);
        double spd4 = SmartDashboard.getNumber("Motor4", 0);

        motor1.setSpeed(spd1);
        motor2.setSpeed(spd2);
        motor3.setSpeed(spd3);
        motor4.setSpeed(spd4);
    }
