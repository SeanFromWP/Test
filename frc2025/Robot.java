package team10034.frc2025;

import edu.wpi.first.wpilibj.TimedRobot;
import team10034.frc2025.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
    private SwerveSubsystem swerveSubsystem;

    @Override
    public void robotInit() {
        swerveSubsystem = new SwerveSubsystem();
    }

    @Override
    public void teleopPeriodic() {
        swerveSubsystem.update();
    }
}