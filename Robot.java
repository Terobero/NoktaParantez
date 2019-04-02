package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.cameraserver.*;

public class Robot extends TimedRobot {
  //Ball Motors
  VictorSP ballBody = new VictorSP(6);
  VictorSP ball1 = new VictorSP(0);
  VictorSP ball2 = new VictorSP(1);

  //Drive Motors
  SpeedControllerGroup left = new SpeedControllerGroup(new VictorSP(3), new VictorSP(2));
  SpeedControllerGroup right = new SpeedControllerGroup(new VictorSP(4), new VictorSP(5));
  private final DifferentialDrive robot = new DifferentialDrive(left, right);
  boolean reverse = false;
  double mod = 1;

  
  //Joystick & Buttons
  private final Joystick stick = new Joystick(0);
  double x_axis, y_axis, z_axis, slider;
  boolean[] buttons = new boolean[13];
  int i;
  int reverseTime = 1000;

  //Gyro stuff
  ADXRS450_Gyro gyro;
  double kP = 0.03;
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  double motorPower;

  //Pneumatics
  Solenoid solenoidDiskBackward = new Solenoid(1);
  Solenoid solenoidDiskForward = new Solenoid(0);
  Solenoid solenoidBallBackward = new Solenoid(2);
  Solenoid solenoidBallForward = new Solenoid(3);
  Compressor compressor = new Compressor(0);
  int diskTime = 1000;
  int ballTime = 1000;

  public void loop() {
    reverseTime ++; if (reverseTime > 1000) reverseTime = 1000;
    diskTime ++; if (diskTime > 1000) diskTime = 1000;
    ballTime ++; if (diskTime > 1000) ballTime = 1000;

    for (int i = 1; i <= 12; i++) buttons[i] = stick.getRawButton(i);
    y_axis = stick.getY();
    x_axis = stick.getX();

    if(buttons[2] && reverseTime > 50) {
      reverseTime = 0;
      reverse = !reverse;
    }
    
    slider = (stick.getRawAxis(3) + 1)/2.0 * 0.9;
    if(buttons[1]) x_axis = 0;
    if(reverse) robot.arcadeDrive(-y_axis*slider, x_axis*slider*0.8);
    else robot.arcadeDrive(y_axis*slider, x_axis*slider*0.8);

    if(buttons[3])ballBody.set(0.8);
    else if(buttons[5]) ballBody.set(-1);
    else ballBody.set(0);

    if(buttons[6]) {ball1.set(0.8);ball2.set(-0.8);}
    else if(buttons[4]) {ball1.set(-0.7);ball2.set(0.7);}
    else {ball1.set(0);ball2.set(0);}  
    
    if(buttons[10]) {
      solenoidDiskForward.set(true);
      diskTime = 0;
    }
    else {
      solenoidDiskForward.set(false);
      if(diskTime > 5*8 && diskTime < 10*8) solenoidDiskBackward.set(true);
      else solenoidDiskBackward.set(false);
    }
    if(buttons[9]) {
      solenoidBallForward.set(true);
      ballTime = 0;
      ball1.set(0.8);
      ball2.set(-0.8);
    }
    else {
      solenoidBallForward.set(false);
      if(ballTime > 50 && ballTime < 100) solenoidBallBackward.set(true);
      else solenoidBallBackward.set(false);
    }

    if(buttons[12]) compressor.start();
    else compressor.stop();
  }

  @Override
  public void robotInit() {
    robot.setSafetyEnabled(false);
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    compressor.setClosedLoopControl(true);
    compressor.stop();

    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override public void autonomousInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override public void autonomousPeriodic() {
    loop();
  }

  @Override
  public void teleopInit() {
    gyro.reset();
    turned = true;
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() {
    loop();

    System.out.println("Motor Speed: " + slider);
  }
  
  @Override  public void testPeriodic() {  }
}
