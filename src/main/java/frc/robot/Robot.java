// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// **************************************************************
// FRC 2022 Rapid React - Team 7454 - Huskies on Hogs
// Mentor: Abram Devonshire - abramdevonshire@gmail.com
// 
// Shift+F5 to deploy
// **************************************************************

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;  //RoboRIO functions
import edu.wpi.first.wpilibj.DriverStation; // ability to get sata from DriverStation
import edu.wpi.first.wpilibj.DigitalInput; // RoboRIO DIO Ports
import edu.wpi.first.wpilibj.AnalogInput; // RoboRio ANALOG IN 0-3
import edu.wpi.first.wpilibj.PneumaticsModuleType; // Interfaces to our Pneumatics Control Module
import edu.wpi.first.wpilibj.Compressor; // ability to use compressor
import edu.wpi.first.wpilibj.DoubleSolenoid; 

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // JOYSTICKS
  private final Joystick flight = new Joystick(0);
  private final Joystick controller = new Joystick(1);  
  // Controller Buttons
  //private static final int aButton = 1;
  //private static final int bButton = 2;
  //private static final int xButton = 3;  
  private static final int yButton = 4;  
  private static final int lbButton = 5; 
  private static final int rbButton = 6; 
  private static final int backButton = 7; 
  private static final int startButton = 8;
  // Flight Buttons
  private static final int flight1 = 1; //trigger
  private static final int flight2 = 2; //thumb
  //private static final int flight3 = 3; 
  private static final int flight4 = 4; 
  //private static final int flight5 = 5; 
  //private static final int flight6 = 6; 
  private static final int flight7 = 7; 
  private static final int flight8 = 8; 
  private static final int flight9 = 9;
  private static final int flight10 = 10;
  private static final int flight11 = 11;
  private static final int flight12 = 12;
  // Flight Other
  private static final int flightPaddle = 3;  

  // DRIVE MOTORS
  // Assign motors to groups, assign groups to Drive
  private final Spark m_frontRioSide = new Spark(0);
  private final Spark m_rearRioSide = new Spark(1);
  private final MotorControllerGroup m_RioSide= new MotorControllerGroup(m_frontRioSide, m_rearRioSide);
  private final Spark m_frontAirSide = new Spark(2);
  private final Spark m_rearAirSide= new Spark(3);
  private final MotorControllerGroup m_AirSide = new MotorControllerGroup(m_frontAirSide, m_rearAirSide);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_RioSide, m_AirSide);
  private double speedMultiplier = 0;
  double xCorrect =.6;  // used to smooth out turning

  // ENCODERS
  Encoder enc_RioSide;
  Encoder enc_AirSide;

  //OTHER MOTORS
  private final Spark m_topShooter = new Spark(4);
  private final Spark m_bottomShooter = new Spark(5);   
  private final Spark m_wheel = new Spark(6);   
  private final MotorControllerGroup m_shooter = new MotorControllerGroup(m_topShooter, m_bottomShooter, m_wheel);


  // DIGITAL INPUT PORTS
  // DigitalInput(0), DigitalInput(1) are mapped to encoder enc_AirSide
  // DigitalInput(2), DigitalInput(3) are mapped to Encoder enc_RioSide
  private final DigitalInput ls_climbAirSide = new DigitalInput(5);
  private final DigitalInput ls_climbRioSide = new DigitalInput(6);
  //private final DigitalInput dio_7 = new DigitalInput(7);
  //private final DigitalInput dio_8 = new DigitalInput(8);
  private final DigitalInput s_ballSensor = new DigitalInput(9); // photoelectric
  
  // COMPRESSOR AND PNEUMATICS
  private final Compressor c_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid solenoidShort = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1); // Short Extend on PCM 0, Retract on PCM 1
  private final DoubleSolenoid solenoidLong = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,2,3); // Long Extend on PCM 2, Retract on PCM 3
  private final DoubleSolenoid solenoidMedium = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,4,5); // Medium Extend on PCM 4, Retract on PCM 5

  // ULTRASONIC SENSOR - // MaxBotix MB1043 Ultrasonic Sensor - www.MaxBotix.com/firstrobotics
  private double voltageScaleFactor = 1;
  private AnalogInput ultrasonicSensor = new AnalogInput(0);
  private double ultrasonicSensorRangeRaw = 0;
  private double ultrasonicSensorRangeInches = 0;

  // GLOBAL VARIABLES
  boolean forwardDriveToggle = true;
  String gameInfoAlliance = "Unknown";
  int gameInfoStation = 0;
  boolean bClimberHooked = false;  
  boolean bClimberAbort = false;
  // autonomous variables
  int autoRunCounter = 0;
  int autoEncoderCounter = 0;
  boolean autoBallShot = false;  
  // int autoSensorDistanceToShoot = 0;
  // int autoDistanceToBackupAfterShoot = 0;
  // double autoBackupTurnAmount = 0;
  // double autoForwardSpeed = 0;
  // double autoBackupSpeed = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Set reverse motor set as inverted and disable safety check
    m_AirSide.setInverted(true);
    m_drive.setSafetyEnabled(false);

    // Init Encoders
    enc_AirSide = new Encoder(1,0, false, Encoder.EncodingType.k2X);
    enc_RioSide = new Encoder(2,3, false, Encoder.EncodingType.k2X);
    enc_AirSide.setDistancePerPulse(Math.PI*6/2625); //distance per pulse is pi* (wheel diameter / counts per revolution)
    enc_RioSide.setDistancePerPulse(Math.PI*6/2625); //distance per pulse is pi* (wheel diameter / counts per revolution)
    
    // Invert shooter motors and disable safety check
    m_shooter.setInverted(true);
    m_bottomShooter.setSafetyEnabled(false);
    m_topShooter.setSafetyEnabled(false);
    
    // Turn Compressor on
    c_compressor.enableDigital();
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Get game info
    gameInfoAlliance = DriverStation.getAlliance().toString();
    gameInfoStation = DriverStation.getLocation();
    SmartDashboard.putString("Alliance", gameInfoAlliance);
    SmartDashboard.putNumber("Station", gameInfoStation);    
    // Encoder Details
    SmartDashboard.putNumber("Encoder Rio Side", enc_RioSide.getDistance());
    SmartDashboard.putNumber("Encoder Air Side", enc_AirSide.getDistance());
    // Ultrasonic Sensor  
    //SmartDashboard.putNumber("Ultrasonic Inches", Math.round(ultrasonicSensorRangeInches*100.0)/100.0);    
    // Drive Helpers
    SmartDashboard.putNumber("xCorrect", xCorrect);
    SmartDashboard.putNumber("speedMultiplier", speedMultiplier); 
    SmartDashboard.putBoolean("Ball Sensor", s_ballSensor.get()); 
    SmartDashboard.putBoolean("Forward Drive", forwardDriveToggle); 
    SmartDashboard.putNumber("Y", flight.getY());
    SmartDashboard.putNumber("X", flight.getX()); 
    // Climber Helpers
    SmartDashboard.putBoolean("LS Airside",  ls_climbAirSide.get()); 
    SmartDashboard.putBoolean("LS RioSide", ls_climbRioSide.get()); 
    SmartDashboard.putBoolean("At Pressure", c_compressor.getPressureSwitchValue());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Reset Incoders when Enabled button is clicked.
    enc_AirSide.reset();
    enc_RioSide.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // ********************************
    // * DRIVE CODE
    // ********************************
    // Drive with arcade drive. Y axis drives forward and backward, and the X turns left and right.
    // Shooter side is the front of the robot.

    speedMultiplier = (((flight.getRawAxis(flightPaddle)*-1)*0.2)+.6);
    m_drive.arcadeDrive(flight.getY()*speedMultiplier, flight.getX()*xCorrect);

    // Flip Logic
    if (flight.getRawButton(flight2)) {
      if (forwardDriveToggle==true) {
        forwardDriveToggle=false;
        m_AirSide.setInverted(false);
        m_RioSide.setInverted(true);
        xCorrect = Math.abs(xCorrect)*-1;
      }
      else {
        forwardDriveToggle=true;
        m_AirSide.setInverted(true);
        m_RioSide.setInverted(false);
        xCorrect = Math.abs(xCorrect);
      }
      // prevent doubleclick of the flip button
      try { 
        Thread.sleep(300); 
      } 
      catch(InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    // ********************************
    // * SHOOTER
    // ********************************
    if (flight.getRawButton(flight1)) {
      m_shooter.set(1);
    }
    else if (controller.getRawButton(rbButton)) { // controller X button
      m_shooter.set(1);
    } 
    else if (controller.getRawButton(lbButton)) { // reverse balls
      m_shooter.set(-1);
    } 
    else {
      m_shooter.set(0);
    }    

    // ********************************
    // * INTAKE
    // ********************************    
    if (controller.getRawButton(yButton)) { // Reverse Dump other team ball
      m_shooter.set(-1);
    } 
      else if (s_ballSensor.get() == true) {
        m_shooter.set(1);
    }
    // if (currentBallColor=="Blue" || currentBallColor=="Red") {
    //   if (currentBallColor != gameInfoAlliance) {
    //     m_shooter.set(1);
    //     wait(2000);
    //     m_shooter.set(0);
    //   }
    // }    

    // ********************************
    // * CLIMBER
    // ********************************
    // if (controller.getRawButton(aButton) == true) {
    //   climbPart1();
    // }    
    // if (controller.getRawButton(startButton) == true) {
    //   climbPart2();    
    // }
    if (controller.getRawButton(backButton) == true && controller.getRawButton(startButton) == true) {
      climbPart1();
      climbPart2();
      if (!bClimberAbort) {
        climbPart3(); 
      }
      else {
        bClimberHooked = false;
      }
    }

    // Small manual
    if (flight.getRawButton(flight7) == true) {
      solenoidShort.set(DoubleSolenoid.Value.kReverse);
      wait(100);
      solenoidShort.set(DoubleSolenoid.Value.kOff);
    } 
    else if (flight.getRawButton(flight8) == true) {
      solenoidShort.set(DoubleSolenoid.Value.kForward);
      wait(100);
      solenoidShort.set(DoubleSolenoid.Value.kOff);
    }
    // Medium manual
    if (flight.getRawButton(flight9) == true) {
      solenoidMedium.set(DoubleSolenoid.Value.kReverse);
      wait(500);
      solenoidMedium.set(DoubleSolenoid.Value.kOff);
    } 
    else if (flight.getRawButton(flight10) == true) {
      solenoidMedium.set(DoubleSolenoid.Value.kForward);
      wait(500);
      solenoidMedium.set(DoubleSolenoid.Value.kOff);
    }
    // Long manual
    if (flight.getRawButton(flight11) == true) {
      solenoidLong.set(DoubleSolenoid.Value.kReverse);
      wait(1000);
      solenoidLong.set(DoubleSolenoid.Value.kOff);
    } 
    else if (flight.getRawButton(flight12) == true) {
      solenoidLong.set(DoubleSolenoid.Value.kForward);
      wait(1000);
      solenoidLong.set(DoubleSolenoid.Value.kOff);
    }

    // ********************************
    // * SENSORS
    // ********************************
    // Ultrasonic Sensor
    // voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
    // ultrasonicSensorRangeRaw = ultrasonicSensor.getValue();
    // ultrasonicSensorRangeInches = ultrasonicSensorRangeRaw * voltageScaleFactor * 0.0492;    

  } // END teleopPeriodic
  
  // ********************************
  // * AUTONOMOUS SECTION
  // ********************************  
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

    // if (gameInfoStation == 1) { 
    //   autoBackupTurnAmount = -.14; // back and to RioSide
    // } 
    // else if (gameInfoStation == 3) { 
    //   autoBackupTurnAmount = .14; // back and to Airside
    // } 
    // else {
    //  autoBackupTurnAmount = 0; // straight back
    //}  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Ultrasonic Sensor
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
    ultrasonicSensorRangeRaw = ultrasonicSensor.getValue();
    ultrasonicSensorRangeInches = ultrasonicSensorRangeRaw * voltageScaleFactor * 0.0492;   

    // writes start block
    if ( autoRunCounter==0) { 
      SmartDashboard.putNumber("Auto Ultrasonic Inches", Math.round(ultrasonicSensorRangeInches*100.0)/100.0);
      //System.out.println("7454: Autonomous will shoot, reverse, based on " + gameInfoStation + " backupY: " +autoBackupTurnAmount );
      System.out.println("7454: Autonomous start ultrasonicSensor at " + ultrasonicSensorRangeInches + "inches");
    }

    // Drive forward until sensor distance is reached
    // if (autoSensorDistanceToShoot && autoRunCounter==0) {
    //   System.out.println("7454: Autonomous Rolling ultrasonicSensor at " + ultrasonicSensorRangeInches + "inches");
    //   m_drive.arcadeDrive(autoForwardSpeed, 0);
    // }

    // Run this code once when distance is reached
    if ( autoRunCounter==0 && !autoBallShot ) {
      // set encoders to 0 first time code is ran
      enc_RioSide.reset();
      enc_AirSide.reset();   

      // drive forward 3 feet
      m_drive.arcadeDrive(-.5, 0);
      wait(3000);
      m_drive.stopMotor(); // Stop Forward
      m_shooter.set(1); // shoot
      wait(1500); // run for a second to make sure clear 
      m_shooter.set(0); // Stop Shoot
      autoBallShot = true;
      enc_RioSide.reset(); // reset encoders
      enc_AirSide.reset();  
    }
    if ( autoRunCounter==0 && autoBallShot ) {

      // drive backwards 7 feet
      if ((enc_RioSide.getDistance() + enc_AirSide.getDistance())/2 < 7) {
        m_drive.arcadeDrive(.55, 0);   
      }
      else {
        autoRunCounter++;
        m_drive.stopMotor();
        System.out.println("7454: Autonomous complete");
      } 
    } 
  
  }  // END Autonomous


  // ********************************
  // * CUSTOM FUNCTIONS
  // ********************************  

  /** This function creates a wait */
  public static void wait(int time){
    try{Thread.sleep(time);
    }
    catch(InterruptedException ex)
    {
      Thread.currentThread().interrupt();
    }
  }

  // This function will raise medium solenoid, push out small, and move robot into posistion using limit sensors
  public void climbPart1() {
    bClimberHooked = false;
    bClimberAbort = false;

    if (!forwardDriveToggle) {
      m_AirSide.setInverted(true);
      m_RioSide.setInverted(false);
      xCorrect = xCorrect*-1;
    }
    solenoidMedium.set(DoubleSolenoid.Value.kReverse);
    wait(150);
    solenoidMedium.set(DoubleSolenoid.Value.kOff);
    solenoidShort.set(DoubleSolenoid.Value.kReverse);
    wait(100);
    solenoidShort.set(DoubleSolenoid.Value.kOff);
  } 

  // This function will get the medium hook on the bar
  public void climbPart2() {     
    while (!bClimberHooked) {
      if (flight.getRawButton(flight4) == true) {  //abort climb
        System.out.println("7454: Climb  - Abort.");
        m_drive.stopMotor();
        bClimberHooked = true;
        bClimberAbort = true;
      }
      else if (ls_climbRioSide.get() && ls_climbAirSide.get()) { // zero touching
        m_drive.arcadeDrive(-.45, 0);
        System.out.println("7454: Climb - no touch");
      } 
      else if (ls_climbRioSide.get() && !ls_climbAirSide.get()) { // AirSide is touching, RioSide is not = turn right
        m_RioSide.set(-.25);
        System.out.println("7454: Climb  - Airside touching, Rioside not");
      } 
      else if (!ls_climbRioSide.get() && ls_climbAirSide.get()) { // RioSide is touching, AirSide is not = turn left
        m_AirSide.set(-.25);
        System.out.println("7454: Climb  - Airside touching, Rioside not");
      } 
      else if (!ls_climbRioSide.get() && !ls_climbAirSide.get()) { // both touching
        System.out.println("7454: Climb  - Hooked!!!");
        m_drive.stopMotor();
        bClimberHooked = true;
      }
    }
  }

  // This function will complete the climb, medium in, long out, short in, long in
  public void climbPart3() {
    solenoidMedium.set(DoubleSolenoid.Value.kForward);
    wait(2000); 
    solenoidMedium.set(DoubleSolenoid.Value.kOff);
    solenoidLong.set(DoubleSolenoid.Value.kReverse);
    wait(2500);
    solenoidLong.set(DoubleSolenoid.Value.kOff);
    wait(3000);
    solenoidShort.set(DoubleSolenoid.Value.kForward);
    wait(500);
    solenoidShort.set(DoubleSolenoid.Value.kOff);
    wait(800);
    solenoidLong.set(DoubleSolenoid.Value.kForward);
    wait(2000);
    solenoidLong.set(DoubleSolenoid.Value.kOff);   
    bClimberHooked = false;
  }



  // ********************************
  // * MODES WE DON"T USE, BUT COULD
  // ********************************  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}  

} // END Robot
