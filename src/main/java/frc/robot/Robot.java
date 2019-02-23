// This is a test program to autonomously drive a robot along a pre-defined path.
// It is based on the 2019 documentation for "PATHWEAVER AND ROBOT PATH PLANNING" 
// which can be found at:
//    http://wpilib.screenstepslive.com/s/currentCS/m/84338
//
// Another useful reference is "Pathfinder for FRC Java" which can for found at:
//    https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Notifier;

// Make sure you install the CTRE vendor library (should be installed locally)
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

// Make sure you install the ADI vendor library
//    http://maven.highcurrent.io/vendordeps/ADIS16448.json
import com.analog.adis16448.frc.ADIS16448_IMU;

// Make sure you install the Pathfinder vendor library from:
//  http://dev.imjac.in/maven/jaci/pathfinder/PathfinderOLD-latest.json
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // -----------------CONSTATNTS -----------------
  // number of encoder counts per wheel revolution
  private static final int k_ticks_per_rev = 4096;
  // diameter of the wheels
  private static final double k_wheel_diameter = 4.0 / 12.0;
  // maximum velocity of the robot
  private static final double k_max_velocity = 10;
  // port numbers for the left and right speed controllers
  private static final int k_left_channel = 0;
  private static final int k_right_channel = 1;

  // name of this path
  private static final String k_path_name = "example";
  
  // Default Talon and encoder constants:
  private static final int PID_LOOP_IDX = 0;
  public static final int SLOT_IDX = 0;
  public static final int TIMEOUT_MS = 30;
  public static boolean SENSOR_PHASE = false;
  public static boolean MOTOR_INVERT = false;

  // Speed controllers for left and right side
  private TalonSRX m_leftTalon;
  private TalonSRX m_rightTalon;
  // Gyro
  private ADIS16448_IMU m_imu;
  
  // Encoder followers for left and right side
  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  // Nofifier - this is what keeps the robot on the path
  private Notifier m_follower_notifier;

  // Command chooser for use in SmartDashboard
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Set up the speed controllers
    m_leftTalon = new TalonSRX(k_left_channel);
    configureTalon(m_leftTalon);
    m_rightTalon = new TalonSRX(k_right_channel);
    configureTalon(m_rightTalon);

    // Set up the gyro
    m_imu = new ADIS16448_IMU();
    m_imu.reset();
    m_imu.calibrate();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    /*
      1.  Create the trajectories for the left and right sides of the drivetrain. This will look for paths in the
          /home/lvuser/deploy/paths folder on the roboRIO. If you choose the output directory in PathWeaver 
          (as per the instructions in the PathWeaver documentation), PathWeaver will automatically place the
          paths in the proper folder. The full filenames for the paths are: 
            /home/lvuser/deploy/paths/PathName.left.pf1.csv
            /home/lvuser/deploy/paths/PathName.right.pf1.csv 
          for the left and right paths.
    */
    Trajectory left_trajectory = null;
    Trajectory right_trajectory = null;
    try {
      left_trajectory  = PathfinderFRC.getTrajectory(k_path_name + ".left");
    }
    catch (IOException e) { 
      System.out.println("Cannot find left trajectory");
    }
    try {
     right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
    }
    catch (IOException e) { 
      System.out.println("Cannot find right trajectory");
    }

    /*
      2.  Create encoder followers from the left and right trajectories. The encoder followers compute
          the motor values based on where the robot is in the path.
    */
    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    /*
      3.  Configure the encoders used by the followers with the number of counts per wheel revolution
          and diameter and PID constants to tune how fast the follower reacts to changes in velocity.
    */
    m_left_follower.configureEncoder(getEncoderPosition(m_leftTalon), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values (kp, ki, kd, kv) on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    
    m_right_follower.configureEncoder(getEncoderPosition(m_rightTalon), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values (kp, ki, kd, kv) on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    /*
      4.  Create the notifier that will regularly call the followPath() method that computes the motor
          speeds and send them to the motors.
    */
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
  }

  /*
    Notififier method that actually drives the motors
    Each delta time (value programmed into the notifier in the previous code segment) gets the current
    wheel speeds for the left and the right side. Use the predicted heading at each point and the actual
    robot heading from the gyro sensor. The difference between the actual and predicted heading is
    the heading error that is factored into the motor speed setting to help ensure the robot tracks the
    path direction.
  */
  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      int leftEncoderPosition = getEncoderPosition(m_leftTalon);
      int rightEncoderPosition = getEncoderPosition(m_rightTalon);

      double left_speed = m_left_follower.calculate(leftEncoderPosition);
      double right_speed = m_right_follower.calculate(rightEncoderPosition);

      //TODO Make sure that this is the proper angle to retrieve from the IMU gyro
      double heading = m_imu.getAngleZ();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn = 0.8 * (-1.0/80.0) * heading_difference;

      m_leftTalon.set(ControlMode.PercentOutput, left_speed + turn);
      m_rightTalon.set(ControlMode.PercentOutput, right_speed - turn);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /*
    Stop the motors at the start of the Teleop period
    After the autonomous period ends and the teleop period begins, be sure to stop the notifier from
    running the followPath() method (above) and stop the motors in case they were still running.
  */
  @Override
  public void teleopInit() {
    m_follower_notifier.stop();
    m_leftTalon.set(ControlMode.PercentOutput, 0);
    m_rightTalon.set(ControlMode.PercentOutput, 0);
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // Configure the talons and the associated encoders
  private void configureTalon(TalonSRX talon){
		// Choose the sensor and sensor direction
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_LOOP_IDX, TIMEOUT_MS);

		// Set appropriately to ensure sensor is positive when output is positive
		talon.setSensorPhase(SENSOR_PHASE);

		// Set based on what direction you want forward/positive to be.
		// This does not affect sensor phase.
		talon.setInverted(MOTOR_INVERT);

		// Set the peak and nominal outputs, 12V means full
		talon.configNominalOutputForward(0, TIMEOUT_MS);
		talon.configNominalOutputReverse(0, TIMEOUT_MS);
		talon.configPeakOutputForward(1, TIMEOUT_MS);
		talon.configPeakOutputReverse(-1, TIMEOUT_MS);
		
		// Lets grab the 360 degree position of the MagEncoder's absolute
		// position, and initially set the relative sensor to match.
		int absolutePosition = talon.getSensorCollection().getPulseWidthPosition();
		// Mask out overflows, keep bottom 12 bits
		absolutePosition &= 0xFFF;
		if (SENSOR_PHASE)	absolutePosition *= -1;
		if (MOTOR_INVERT)	absolutePosition *= -1;
		
		// Set the quadrature (relative) sensor to match absolute
		talon.setSelectedSensorPosition(absolutePosition, PID_LOOP_IDX, TIMEOUT_MS);
  }

  // Get current position of the encoder on the associated talon
  private int getEncoderPosition(TalonSRX talon) {
    //TODO Make sure that this is the proper method to get the current position in ticks
    int ticks = talon.getSensorCollection().getQuadraturePosition();
    return ticks;
  }
}
