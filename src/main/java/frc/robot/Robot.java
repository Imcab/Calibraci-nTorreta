// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  int DutyPortTorreta = 1;

  double DutyCycleEncoderOffSet = 0.0; 

  DutyCycleEncoder EncoderTorreta =  new DutyCycleEncoder(DutyPortTorreta);

  //Modificar valores para el PID
  double kp = 0;
  double ki = 0;
  double kd = 0;

  //Crear el controlador PID
  PIDController pid = new PIDController(kp,ki,kd);

  //Crear el control de xbox para mover la torreta con pid
  XboxController controller = new XboxController(0);

  //id del motor
  int id = 0;

  //Motor a probar
  CANSparkMax turret = new CANSparkMax(id, MotorType.kBrushless);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    EncoderTorreta.setPositionOffset(Units.degreesToRotations(DutyCycleEncoderOffSet)); 
  }
  
  //Función que regresa la posición de la torreta, para no tener que declarar "turretPosition" en cada modo del robot
  public double getTurretPosition(){
      return Units.rotationsToDegrees(EncoderTorreta.getAbsolutePosition());
  }

  @Override
  public void robotPeriodic() {

    //Pone la torreta en la dashboard
    SmartDashboard.putNumber( "Posición de la torreta", getTurretPosition());

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  //Función para mover la torreta a un ángulo determinado (en grados)
  // - Cambiar a rotaciones si sale mal xd
  public void runSetpoint(double angle){
    turret.set(pid.calculate(getTurretPosition(), angle));
  }

  @Override
  public void teleopPeriodic() {

    //Mueve la torreta para prueba del pid
    if (controller.getAButton()) {
      runSetpoint(180);
    }else{
      turret.set(0);
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
