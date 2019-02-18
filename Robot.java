package frc.robot;

import java.nio.charset.StandardCharsets;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  VictorSP ballBody = new VictorSP(6);
  VictorSP ball1 = new VictorSP(0);
  VictorSP ball2 = new VictorSP(1);

  SpeedControllerGroup left = new SpeedControllerGroup(new VictorSP(3), new VictorSP(2));
  SpeedControllerGroup right = new SpeedControllerGroup(new VictorSP(4), new VictorSP(5));
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(left, right);
  private final Joystick m_stick = new Joystick(0);

  double axis1, axis2, axis3;


  ADXRS450_Gyro gyro;
  double kP = 0.03;
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  double motorPower;

  Solenoid solenoid1Backward = new Solenoid(1);
  Solenoid solenoid1Forward = new Solenoid(6);
  Solenoid solenoid2Backward = new Solenoid(2);
  Solenoid solenoid2Forward = new Solenoid(5);
  Solenoid s3f = new Solenoid(0);
  Solenoid s3b = new Solenoid(7);
  Compressor compressor = new Compressor(0);
  int diskTime = 1000;

  NetworkTable table;
  NetworkTableEntry i;

  /* yoel kod */
  //SerialPort raspi = new SerialPort(9600, SerialPort.Port.kOnboard);
  /* /yoel kod */
  
	public void gearControlPneumatics(DoubleSolenoid solenoid, boolean isPressed, boolean take) {
		if(isPressed && take)
			solenoid.set(DoubleSolenoid.Value.kForward);
			
		else if(isPressed && !take)
			solenoid.set(DoubleSolenoid.Value.kReverse);
			
		else
			solenoid.set(DoubleSolenoid.Value.kOff);
	}

  @Override
  public void robotInit() {
    m_robotDrive.setSafetyEnabled(false);
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();

    table = NetworkTableInstance.getDefault().getTable("datatable");
    i = table.getEntry("i");
    i.addListener(event -> {
      System.out.println(event.value.getValue());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {  }

  @Override
  public void autonomousPeriodic() {  }

  @Override
  public void teleopInit() {
    gyro.reset();
    turned = true;
  }

  
  /*public void turnDegrees(int degree) {
    if(turned)return;
    angle = Math.abs(gyro.getAngle() % 360);
    if(degree == 0 && angle > 180){
      angle = Math.abs(180 - ((angle + 180) % 360));
      motorPower = Math.abs((degree-angle)*kP);
      motorPower = 1/(1+Math.pow(Math.E, -motorPower)) * 0.8; //sigmoid
      m_robotDrive.arcadeDrive(m_stick.getY()*0.8, motorPower);
      return;
    }
    motorPower = Math.abs((degree-angle)*kP);
    motorPower = 1/(1+Math.pow(Math.E, -motorPower)) * 0.8; //sigmoid
    System.out.println(motorPower);
    if(angle-2 > degree)m_robotDrive.arcadeDrive(m_stick.getY()*0.8, -motorPower);
    else if(angle+2 < degree)m_robotDrive.arcadeDrive(m_stick.getY()*0.8, motorPower);
    //else turned = true;
  }*/

  public void turnDegrees(int degree) {
    if(turned)return;
    angle = gyro.getAngle();
    motorPower = Math.abs((degree-angle)*kP);
    motorPower = 1/(1+Math.pow(Math.E, -motorPower)) * 0.8; //sigmoid

    if(angle-2 > degree)m_robotDrive.arcadeDrive(m_stick.getY()*0.8, -motorPower);
    else if(angle+2 < degree)m_robotDrive.arcadeDrive(m_stick.getY()*0.8, motorPower);
    else turned = true;
  }

  @Override
  public void teleopPeriodic() {
    axis3 = (m_stick.getRawAxis(3) + 1)/2.0;
    m_robotDrive.arcadeDrive(-m_stick.getY()*0.8, m_stick.getX()*0.8);

    
    if(m_stick.getRawButton(1))turned = true;
    if(m_stick.getPOV() != -1){
      turned = false;
      mustTurnDegree = m_stick.getPOV();
      gyro.reset();
    }
    if(!turned)turnDegrees(mustTurnDegree);
    if(m_stick.getRawButton(2))gyro.reset();

    if(m_stick.getRawButton(3))ballBody.set(axis3);
    else if(m_stick.getRawButton(4))ballBody.set(-axis3);
    else ballBody.set(0);

    if(m_stick.getRawButton(5)) {
      ball1.set(axis3);
      ball2.set(-axis3);
    }
    else if(m_stick.getRawButton(6)) {
      ball1.set(-axis3);
      ball2.set(axis3);
    }
    else {
      ball1.set(0);
      ball2.set(0);
    }

    
    diskTime ++;
    if (diskTime > 1000) diskTime = 1000;
    if(m_stick.getRawButton(10)) {
      solenoid1Forward.set(true);
      solenoid2Forward.set(true);
      s3f.set(true);
      diskTime = 0;
    }
    else {
      solenoid1Forward.set(false);
      solenoid2Forward.set(false);
      s3f.set(false);
      if(diskTime > 50 && diskTime < 100) {
        solenoid1Backward.set(true);
        solenoid2Backward.set(true);
        s3b.set(true);
      }
      else {
        solenoid1Backward.set(false);
        solenoid2Backward.set(false);
        s3b.set(false);
      }
    }
  }
  
  @Override  public void testPeriodic() {
    if(this.raspi.getBytesReceived() > 0)
      System.out.println(new String(this.raspi.read(this.raspi.getBytesReceived())));
   }
}
