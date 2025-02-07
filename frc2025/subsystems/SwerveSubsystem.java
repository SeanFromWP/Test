package team10034.frc2025.subsystems;

import team10034.frc2025.Constants.ConvertConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;

public class SwerveSubsystem {
    private final SwerveModule lF;
    private final SwerveModule rF;
    private final SwerveModule lB;
    private final SwerveModule rB;
    private final XboxController joystick;

    public SwerveSubsystem() {
        lF = new SwerveModule(21, 11, 86, 31);
        rF = new SwerveModule(22, 12, 185, 32);
        lB = new SwerveModule(23, 13, 212, 33);
        rB = new SwerveModule(24, 14, 20, 34);
        joystick = new XboxController(0);
    }

    public void update() {
        double targetAngle = Math.atan2(-joystick.getLeftY(), joystick.getLeftX()) * 180 / Math.PI;
        targetAngle = (targetAngle + 630) % 360;
        double targetSpeed = Math.hypot(joystick.getLeftX(), joystick.getLeftY()) * 5676;
        double rot = joystick.getRightY();

        SwerveModuleState[] states = combineMoveAndRotStates(new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle)), rot);

        lF.setTargetAngle(states[0].angle.getDegrees());
        rF.setTargetAngle(states[1].angle.getDegrees());
        lB.setTargetAngle(states[2].angle.getDegrees());
        rB.setTargetAngle(states[3].angle.getDegrees());

        lF.setTargetSpeed(states[0].speedMetersPerSecond);
        rF.setTargetSpeed(states[1].speedMetersPerSecond);
        lB.setTargetSpeed(states[2].speedMetersPerSecond);
        rB.setTargetSpeed(states[3].speedMetersPerSecond);

        SmartDashboard.putNumber("lF Angle", lF.getAbsoluteAngle());
        SmartDashboard.putNumber("rF Angle", rF.getAbsoluteAngle());
        SmartDashboard.putNumber("lB Angle", lB.getAbsoluteAngle());
        SmartDashboard.putNumber("rB Angle", rB.getAbsoluteAngle());
        SmartDashboard.putNumber("lF Speed", lF.getSpeed());
        SmartDashboard.putNumber("rF Speed", rF.getSpeed());
        SmartDashboard.putNumber("lB Speed", lB.getSpeed());
        SmartDashboard.putNumber("rB Speed", rB.getSpeed());
    }

    public static SwerveModuleState[] combineMoveAndRotStates(SwerveModuleState movingState, double rot){
        SwerveModuleState leftFrontTurn = new SwerveModuleState(1, Rotation2d.fromDegrees(45));
        SwerveModuleState rightFrontTurn = new SwerveModuleState(1, Rotation2d.fromDegrees(135));
        SwerveModuleState leftBackTurn = new SwerveModuleState(1, Rotation2d.fromDegrees(225));
        SwerveModuleState rightBackTurn = new SwerveModuleState(1, Rotation2d.fromDegrees(315));

        /**
         * XY軸的運動狀態 + 
         * 機器人旋轉的角速度 * 輪子的offset
         * = 輪子應該到的運動狀態
         */

        SwerveModuleState leftFrontDesiredState = plus(movingState, multiplyState(leftFrontTurn, rot));
        SwerveModuleState rightFrontDesiredState = plus(movingState, multiplyState(rightFrontTurn, rot));
        SwerveModuleState leftBackDesiredState = plus(movingState, multiplyState(leftBackTurn, rot));
        SwerveModuleState rightBackDesiredState = plus(movingState, multiplyState(rightBackTurn, rot));

        return new SwerveModuleState[] {leftFrontDesiredState, rightFrontDesiredState, leftBackDesiredState, rightBackDesiredState};
    }

    public static SwerveModuleState plus(SwerveModuleState a, SwerveModuleState b) {
        // 極座標轉直角坐標
        double xA = a.speedMetersPerSecond * a.angle.getCos();
        double yA = a.speedMetersPerSecond * a.angle.getSin();
        
        double xB = b.speedMetersPerSecond * b.angle.getCos();
        double yB = b.speedMetersPerSecond * b.angle.getSin();
    
        // 直角坐標加法
        double resultX = xA + xB;
        double resultY = yA + yB;
    
        // 直角坐標轉極座標
        double resultSpeed = Math.hypot(resultX, resultY);
        double resultAngle = ConvertConstants.kAtan2To360(Math.atan2(resultY, resultX));
    
        return new SwerveModuleState(resultSpeed, new Rotation2d(resultAngle));
    }
    
    public static SwerveModuleState multiplyState(SwerveModuleState state, double scale){
        return new SwerveModuleState(state.speedMetersPerSecond * scale, state.angle);
    }
}
