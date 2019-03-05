package frc.robot;

import org.opencv.core.Mat;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
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

  //Sensors
  Potentiometer pot = new AnalogPotentiometer(0, 360, 0);

  @Override
  public void robotInit() {
    robot.setSafetyEnabled(false);
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    compressor.setClosedLoopControl(true);

    CameraServer.getInstance().startAutomaticCapture();
    /*
    new Thread (() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(1280, 720);
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource ouputStream =  CameraServer.getInstance().putVideo("Front Camera", 640, 480);
      Mat source = new Mat();
      Mat sink = new Mat();

     while(!Thread.interrupted()){
        cvSink.grabFrame(source);

     }
   }).start();*/
  }

  @Override public void autonomousInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override public void autonomousPeriodic() {  }

  @Override
  public void teleopInit() {
    gyro.reset();
    turned = true;
    CameraServer.getInstance().startAutomaticCapture();
  }

  
  public void turnDegrees(int degree) {
    if(turned)return;
    angle = Math.abs(gyro.getAngle() % 360);
    if(degree == 0 && angle > 180){
      angle = Math.abs(180 - ((angle + 180) % 360));
      motorPower = Math.abs((degree-angle)*kP);
      motorPower = 1/(1+Math.pow(Math.E, -motorPower)) * mod; //sigmoid
      robot.arcadeDrive(stick.getY()*mod, motorPower);
      return;
    }
    motorPower = Math.abs((degree-angle)*kP);
    motorPower = 1/(1+Math.pow(Math.E, -motorPower)) * mod; //sigmoid
    System.out.println(motorPower);
    if(angle-2 > degree)robot.arcadeDrive(stick.getY()*mod, -motorPower);
    else if(angle+2 < degree)robot.arcadeDrive(stick.getY()*mod, motorPower);
    //else turned = true;
  }

  @Override
  public void teleopPeriodic() {
    reverseTime ++; if (reverseTime > 1000) reverseTime = 1000;
    diskTime ++; if (diskTime > 1000) diskTime = 1000;
    ballTime ++; if (diskTime > 1000) ballTime = 1000;
    for (int i = 1; i <= 12; i++)
      buttons[i] = stick.getRawButton(i);
   
    if(buttons[1]) mod = 0.6;
    else mod = 1.0;

    if(buttons[2]) reverse = !reverse;
    
    slider = (stick.getRawAxis(3) + 1)/2.0;

    if(reverse) robot.arcadeDrive(y_axis, -x_axis);
    else robot.arcadeDrive(y_axis, x_axis);

    
    if(buttons[1])turned = true;
    if(stick.getPOV() != -1){
      turned = false;
      mustTurnDegree = stick.getPOV();
      gyro.reset();
    }
    if(!turned)turnDegrees(mustTurnDegree);

    if(buttons[3])ballBody.set(slider);
    else if(buttons[5] && pot.get() > 170)ballBody.set(-slider);
    else ballBody.set(0);

    if(buttons[6]) {
      ball1.set(slider);
      ball2.set(-slider);
    }
    else if(buttons[4]) {
      ball1.set(-slider);
      ball2.set(slider);
    }
    else {
      ball1.set(0);
      ball2.set(0);
    }

    
    
    
    if(buttons[10]) {
      solenoidDiskForward.set(true);
      diskTime = 0;
    }
    else {
      solenoidDiskForward.set(false);
      if(diskTime > 50 && diskTime < 100) solenoidDiskBackward.set(true);
      else solenoidDiskBackward.set(false);
    }

    
    
    if(buttons[9]) {
      solenoidBallForward.set(true);
      ballTime = 0;
      ball1.set(slider);
      ball2.set(-slider);
    }
    else {
      solenoidBallForward.set(false);
      if(ballTime > 50 && ballTime < 100) solenoidBallBackward.set(true);
      else {
        solenoidBallBackward.set(false);
      }
    }

    System.out.println(pot.get());
  }
  
  @Override  public void testPeriodic() {  }
}
