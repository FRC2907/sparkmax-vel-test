// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  CANSparkMax fl, fr, bl, br;
  CANSparkMax[] drive_motors = {fl, fr, bl, br};
  SparkPIDController[] drive_controllers = {
    fl.getPIDController()
    , fr.getPIDController()
    , bl.getPIDController()
    , br.getPIDController()
  };
  RelativeEncoder[] drive_encoders = {
    fl.getEncoder()
    , fr.getEncoder()
    , bl.getEncoder()
    , br.getEncoder()
  };

  @Override
  public void robotInit() {
    fl = new CANSparkMax(1, MotorType.kBrushless);
    fr = new CANSparkMax(2, MotorType.kBrushless);
    bl = new CANSparkMax(3, MotorType.kBrushless);
    br = new CANSparkMax(4, MotorType.kBrushless);
    for (CANSparkMax motor : drive_motors) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBrake);
      motor.setInverted(false);
    }
    for (RelativeEncoder enc : drive_encoders) {
      enc.setInverted(false); // maybe these need to be inverted on inverted motors?
      enc.setPosition(0.0);
      enc.setPositionConversionFactor(0.0);
      enc.setVelocityConversionFactor(kDefaultPeriod);
    }
    for (SparkPIDController ctl : drive_controllers) {
      ctl.setP(0.5, 0);
      ctl.setI(0, 0);
      ctl.setD(0, 0);
      ctl.setFF(0, 0);
      //ctl.setFeedbackDevice(null); // may be necessary
      ctl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
      ctl.setSmartMotionAllowedClosedLoopError(1.0, 0);
      ctl.setSmartMotionMaxAccel(10.0, 0);
      ctl.setSmartMotionMaxVelocity(1000.0, 0);
    }
    fr.setInverted(true);
    br.setInverted(true);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    for (SparkPIDController ctl : drive_controllers) {
      ctl.setReference(500.0, ControlType.kSmartVelocity, 0); // try also kVelocity
    }
    for (CANSparkMax motor : drive_motors) {
      // metrics
      int id  = motor.getDeviceId();
      double u = motor.getAppliedOutput() * motor.getBusVoltage();
      double v = motor.getEncoder().getVelocity();
      double input_per_velocity = u / v;
      SmartDashboard.putNumber("    u_" + id, u);
      SmartDashboard.putNumber("    v_" + id, v);
      SmartDashboard.putNumber("(u/v)_" + id, input_per_velocity);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
