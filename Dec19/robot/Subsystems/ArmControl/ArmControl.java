package frc.robot.Subsystem.ArmControl;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmControl extends SubsystemBase{

    /* 定義零件 */

    private final CANSparkMax armTurner1; 
    private final CANSparkMax armTurner2;
    private final CANSparkMax armTurner3; 
    private final CANSparkMax armTurner4; 
    private final CANSparkMax armTurner5;
    private final CANSparkMax armTurner6;
    private final CANSparkMax armJoint1;
    private final CANSparkMax armJoint2;
    private final CANSparkMax elbowJoint1;
    private final CANSparkMax elbowJoint2;

    /*
    private final AbsoluteEncoder armTurnerEncoder1;
    private final AbsoluteEncoder armTurnerEncoder2;
    private final AbsoluteEncoder armTurnerEncoder3; 
    private final AbsoluteEncoder armTurnerEncoder4; 
    private final AbsoluteEncoder armTurnerEncoder5;
    private final AbsoluteEncoder armTurnerEncoder6;
    private final AbsoluteEncoder armJointEncoder1;
    private final AbsoluteEncoder armJointEncoder2;
    private final AbsoluteEncoder elbowJointEncoder1;
    private final AbsoluteEncoder elbowJointEncoder2;
    */

    private final Talon hand;

    /* 速度設定 */
    //要改

    private final double raiseRPM = 400;//TODO 要調
    private final double turnRPM = 400;//TODO 要調
    private final double elbowRPM = 400;//TODO 要調


    /* 距離數值項（瞄準用） */
    private final double maxDistance = 200;//TODO 要調
    private final double minDistance = 20;//TODO 要調

    /* 編碼器數值項 */

    private final double armTo90DegreesPosition = 1000; //TODO 要調
    private final double elbowTo45DegreesPosition = 1000; //TODO 要調
    private final double convertAngleToEncoder = 4096 / 360; //拿來轉換角度到編碼器

    /* 機器人配置項 */

    private final double armInitHeight = 46;
    private final double armLength = 52;
    private final double elbowAndHandLength = 59; //TODO 要調
    private final double elbowLength = 55;

    /* 比賽常數項 */
    /* Speaker */
    private final double speakerGateHeight = 198;
    /* AMP */
    private final double AMPHeight = 112; //應該是112，但是先試試看

    public ArmControl(int armTurner1ID, int armTurner2ID, int armTurner3ID,
    int armTurner4ID, int armTurner5ID, int armTurner6ID,
    int armJoint1ID, int armJoint2ID, int elbowJoint1ID, int elbowJoint2ID, int handPWM) {

    hand = new Talon(handPWM);

    armTurner1  = createArmMotor(armTurner1ID);
    armTurner2  = createArmMotor(armTurner2ID);
    armTurner3  = createArmMotor(armTurner3ID);
    armTurner4  = createArmMotor(armTurner4ID);
    armTurner5  = createArmMotor(armTurner5ID);
    armTurner6  = createArmMotor(armTurner6ID);
    armJoint1   = createArmMotor(armJoint1ID);
    armJoint2   = createArmMotor(armJoint2ID);
    elbowJoint1   = createArmMotor(elbowJoint1ID);
    elbowJoint2   = createArmMotor(elbowJoint2ID);
    elbowJoint2.setInverted(true); //就他是反的
/*
    armTurnerEncoder1  = armTurner1.getAbsoluteEncoder();
    armTurnerEncoder2  = armTurner2.getAbsoluteEncoder();
    armTurnerEncoder3  = armTurner3.getAbsoluteEncoder();
    armTurnerEncoder4  = armTurner4.getAbsoluteEncoder();
    armTurnerEncoder5  = armTurner5.getAbsoluteEncoder();
    armTurnerEncoder6  = armTurner6.getAbsoluteEncoder();
    armJointEncoder1  = armJoint1.getAbsoluteEncoder();
    armJointEncoder2  = armJoint2.getAbsoluteEncoder(); 
    elbowJointEncoder1  = elbowJoint1.getAbsoluteEncoder();
    elbowJointEncoder2  = elbowJoint2.getAbsoluteEncoder();*/

    armTurner1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armTurner2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armTurner3.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armTurner4.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armTurner5.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armTurner6.setIdleMode(CANSparkMax.IdleMode.kBrake);

    armJoint1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armJoint2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    elbowJoint1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elbowJoint2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }


    private CANSparkMax createArmMotor(int canID) {
        return new CANSparkMax(canID, CANSparkMax.MotorType.kBrushless);
    }

    /* 控制函式 */
    /* 控制臂部 */
    /* 左右旋轉臂部 */
    public void turnArm(double speed) {
        armTurner1.getPIDController().setReference(turnRPM * speed, CANSparkMax.ControlType.kVelocity);
        armTurner2.getPIDController().setReference(turnRPM * speed, CANSparkMax.ControlType.kVelocity);
        armTurner3.getPIDController().setReference(turnRPM * speed, CANSparkMax.ControlType.kVelocity);
        armTurner4.getPIDController().setReference(turnRPM * speed, CANSparkMax.ControlType.kVelocity);
        armTurner5.getPIDController().setReference(turnRPM * speed, CANSparkMax.ControlType.kVelocity);
        armTurner6.getPIDController().setReference(turnRPM * speed, CANSparkMax.ControlType.kVelocity);
    }
    /* 上下彎曲臂部 */
    public void raiseArm(double speed) {
        armJoint1.getPIDController().setReference(raiseRPM * speed, CANSparkMax.ControlType.kVelocity);
        armJoint2.getPIDController().setReference(raiseRPM * speed, CANSparkMax.ControlType.kVelocity);
    }

    /* 控制肘部 */
    public void turnElbow(Boolean bend, Boolean raise) {
        if (bend == true && raise == false){
            elbowJoint1.getPIDController().setReference(elbowRPM, CANSparkMax.ControlType.kVelocity);
            elbowJoint2.getPIDController().setReference(elbowRPM, CANSparkMax.ControlType.kVelocity);
        }
        else if (bend == false && raise == true){
            elbowJoint1.getPIDController().setReference(elbowRPM, CANSparkMax.ControlType.kVelocity);
            elbowJoint2.getPIDController().setReference(elbowRPM, CANSparkMax.ControlType.kVelocity);
        }
        else {
            elbowJoint1.set(0);
            elbowJoint2.set(0);
        }
    }
    /* 控制手 */
    public void controlHand(Boolean hold, Boolean release) {
        if (hold == true && release == false){
            hand.set(0);
        }
        else if (hold == false && release == true){
            hand.set(1);
        }
        else {
            hand.set(0);
        }
    }

    /* 瞄準函式 */
    /* 瞄準AMP */
    public void aimAMP(double distanceToAMP){
        if (distanceToAMP < maxDistance && distanceToAMP > minDistance){
            double diagonalOfTriangle = Math.sqrt(Math.pow(AMPHeight - armInitHeight, 2) + Math.pow(distanceToAMP, 2));
            //畢氏定理

            /*上面三角形的theta */
            double upper = lawOfCosineGetTheta(elbowAndHandLength, diagonalOfTriangle, armLength);

            /*下面三角形的theta */
            double lower = Math.acos(diagonalOfTriangle/distanceToAMP);
            double armTheta = upper + lower; // 肘長的對角
            double elbowTheta = lawOfCosineGetTheta(diagonalOfTriangle, elbowAndHandLength, armLength); //下面斜邊長的對角
            setArmDegree(90-armTheta);
            setElbowDegree(elbowTheta);
        }
    }

    /* 瞄準Speaker*/

    //計算完美發射距離（最好對到的地方）
    private double calculateIdealSpeakerTheta(){
        //這些看12/19的草稿看得到
        double idealDistance = 193; //TODO 這個約是白線的距離
        double c = Math.sqrt(Math.pow(idealDistance,2) + Math.pow(speakerGateHeight,2));

        double anglePhi_1 = Math.acos(elbowAndHandLength/c);
        double anglePhi_2 = Math.acos(idealDistance/c);
        double idealSpeakerTheta = 90 - anglePhi_1 - anglePhi_2;
        return idealSpeakerTheta;
    }

    /* 12/19決定先鎖死數值，因為紅外線距離儀不能準確校正 */
    public void aimSpeaker(){
        double targetSpeakerDeltaAngle = calculateIdealSpeakerTheta();
        double targetSpeakerDeltaPosition = targetSpeakerDeltaAngle * convertAngleToEncoder;

        armJoint1.getPIDController().setReference(targetSpeakerDeltaPosition, CANSparkMax.ControlType.kVelocity);
        armJoint2.getPIDController().setReference(targetSpeakerDeltaPosition, CANSparkMax.ControlType.kVelocity);
        elbowJoint1.getPIDController().setReference(elbowTo45DegreesPosition, CANSparkMax.ControlType.kVelocity);
        elbowJoint2.getPIDController().setReference(elbowTo45DegreesPosition, CANSparkMax.ControlType.kVelocity);
    }

    /* 控制手臂角度 */

    private void setArmDegree(double deltaTo90){
        double deltaPosition = deltaTo90 * convertAngleToEncoder; //先找出離手臂垂直的delta，換成編碼器數值
        double targetArmPosition = ((armTo90DegreesPosition + deltaPosition) % 4096); //設定目標編碼器數值
        armJoint1.getPIDController().setReference(targetArmPosition, CANSparkMax.ControlType.kVelocity); //跑下去，這裡只有1個馬達，要改
    }

    private void setElbowDegree(double deltaTo45){
        double deltaPosition = deltaTo45 * convertAngleToEncoder;
        double targetArmPosition = ((elbowTo45DegreesPosition + deltaPosition) % 4096);
        elbowJoint1.getPIDController().setReference(targetArmPosition, CANSparkMax.ControlType.kVelocity);
    }

    /* 數學公式 */
    // 用餘弦定理來計算目標角度，return是角度
    private double lawOfCosineGetTheta(double targetSide, double sideA, double sideB){
        double thisAngleNeedsToAcos = (
        (Math.pow(sideA, 2) + Math.pow(sideB, 2) - Math.pow(targetSide, 2)) / (2 * sideA * sideB)
        ); //餘弦定理, cos(c) = a^2 + b^2 - c^2 / 2ab
        double targetThetaAngle = Math.toDegrees(Math.acos(thisAngleNeedsToAcos)); // acos，然後換成角度
        return targetThetaAngle;
    }
  }
