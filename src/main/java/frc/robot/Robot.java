// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;


//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Subsystem.DriveTrainSubsystem;

//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

//import com.ctre.phoenix.motorcontrol.can.*;
//import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private DriveTrainSubsystem driveTrain;

  Joystick leftStick;
  Joystick rightStick;

  CANSparkMax leftFrontMotor;
  CANSparkMax leftBackMotor;
  CANSparkMax rightFrontMotor;
  CANSparkMax rightBackMotor;

  CANSparkMax leftFrontTurn;
  CANSparkMax leftBackTurn;
  CANSparkMax rightFrontTurn;
  CANSparkMax rightBackTurn;
  
  RelativeEncoder leftFrontEncoder;
  RelativeEncoder leftBackEncoder;
  RelativeEncoder rightFrontEncoder;
  RelativeEncoder rightBackEncoder;

  RelativeEncoder leftFrontTurnEncoder;
  RelativeEncoder leftBackTurnEncoder;
  RelativeEncoder rightFrontTurnEncoder;
  RelativeEncoder rightBackTurnEncoder;

  double speed = 1;

  double leftPosition;
  double rightPosition;
  double leftTurnPosition;
  double rightTurnPosition;
  String leftMotorPos;
  String leftTurnPos;
  String rightMotorPos;
  String rightTurnPos;

  final int leftFrontMotorID = 28;
  final int leftFrontTurnID = 27;
  final int leftBackMotorID = 26;
  final int leftBackTurnID = 25;
  final int rightFrontMotorID = 22;
  final int rightFrontTurnID = 21;
  final int rightBackMotorID = 24;
  final int rightBackTurnID = 23;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Defalt Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftStick = new Joystick(0);
    driveTrain = new DriveTrainSubsystem(0, 1, leftFrontMotorID, leftBackMotorID, rightFrontMotorID, rightBackMotorID, leftFrontTurnID, leftBackTurnID, rightFrontTurnID, rightBackTurnID); // CAN ID和操纵杆端口示例
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { 
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    driveTrain.moveWithPID(20, speed);
    /*
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double speed2 = leftStick.getRawAxis(1);
 /*   double xaxis = leftStick.getRawAxis(0);
    double yaxis = leftStick.getRawAxis(1);
    double xaxis2 = leftStick.getRawAxis(3);*/
    driveTrain.moveWithPID(20, speed2);
    driveTrain.displayEncoderValues();
  }

  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static String getKdefaultauto() {
    return kDefaultAuto;
  }

  public static String getKcustomauto() {
    return kCustomAuto;
  }

  public String getM_autoSelected() {
    return m_autoSelected;
  }

  public void setM_autoSelected(String m_autoSelected) {
    this.m_autoSelected = m_autoSelected;
  }

  public SendableChooser<String> getM_chooser() {
    return m_chooser;
  }

  public Joystick getLeftStick() {
    return leftStick;
  }

  public void setLeftStick(Joystick leftStick) {
    this.leftStick = leftStick;
  }

  public Joystick getRightStick() {
    return rightStick;
  }

  public void setRightStick(Joystick rightStick) {
    this.rightStick = rightStick;
  }

  public CANSparkMax getLeftFrontMotor() {
    return leftFrontMotor;
  }

  public void setLeftFrontMotor(CANSparkMax leftFrontMotor) {
    this.leftFrontMotor = leftFrontMotor;
  }

  public CANSparkMax getLeftBackMotor() {
    return leftBackMotor;
  }

  public void setLeftBackMotor(CANSparkMax leftBackMotor) {
    this.leftBackMotor = leftBackMotor;
  }

  public CANSparkMax getRightFrontMotor() {
    return rightFrontMotor;
  }

  public void setRightFrontMotor(CANSparkMax rightFrontMotor) {
    this.rightFrontMotor = rightFrontMotor;
  }

  public CANSparkMax getRightBackMotor() {
    return rightBackMotor;
  }

  public void setRightBackMotor(CANSparkMax rightBackMotor) {
    this.rightBackMotor = rightBackMotor;
  }

  public RelativeEncoder getLeftFrontEncoder() {
    return leftFrontEncoder;
  }

  public void setLeftFrontEncoder(RelativeEncoder leftFrontEncoder) {
    this.leftFrontEncoder = leftFrontEncoder;
  }

  public RelativeEncoder getLeftBackEncoder() {
    return leftBackEncoder;
  }

  public void setLeftBackEncoder(RelativeEncoder leftBackEncoder) {
    this.leftBackEncoder = leftBackEncoder;
  }

  public RelativeEncoder getRightFrontEncoder() {
    return rightFrontEncoder;
  }

  public void setRightFrontEncoder(RelativeEncoder rightFrontEncoder) {
    this.rightFrontEncoder = rightFrontEncoder;
  }

  public RelativeEncoder getRightBackEncoder() {
    return rightBackEncoder;
  }

  public void setRightBackEncoder(RelativeEncoder rightBackEncoder) {
    this.rightBackEncoder = rightBackEncoder;
  }

  public CANSparkMax getLeftFrontTurn() {
    return leftFrontTurn;
  }

  public void setLeftFrontTurn(CANSparkMax leftFrontTurn) {
    this.leftFrontTurn = leftFrontTurn;
  }

  public CANSparkMax getLeftBackTurn() {
    return leftBackTurn;
  }

  public void setLeftBackTurn(CANSparkMax leftBackTurn) {
    this.leftBackTurn = leftBackTurn;
  }

  public CANSparkMax getRightFrontTurn() {
    return rightFrontTurn;
  }

  public void setRightFrontTurn(CANSparkMax rightFrontTurn) {
    this.rightFrontTurn = rightFrontTurn;
  }

  public CANSparkMax getRightBackTurn() {
    return rightBackTurn;
  }

  public void setRightBackTurn(CANSparkMax rightBackTurn) {
    this.rightBackTurn = rightBackTurn;
  }
}
