package com.fairportfirst.frc2012.drive;

import com.fairportfirst.frc2012.structures.*;
import com.fairportfirst.frc2012.vision.Camera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The supreme central class whence all other classes are called
 * @author brennan
 */
public class MainDrive extends IterativeRobot {

    // Statics for CANJaguar
    private static int JAG_LEFT_A = 2;
    private static int JAG_LEFT_B = 4;
    private static int JAG_RIGHT_A = 3;
    private static int JAG_RIGHT_B = 5;
    private static int JAG_SHOOTER_WHEELS_A = 6;
    private static int JAG_SHOOTER_WHEELS_B = 7;
    private static int JAG_TURRET = 9;

    // Statics for PWM (Victors)
    private static int VIC_WEDGE = 1;
    private static int VIC_LIFT = 2;
    private static int VIC_BALLPICKUP = 3;

    // Statics for Digital I/O (Limit Switches)
//    private static int DIO_TURRET_CW_LIMIT = 1;
//    private static int DIO_TURRET_CCW_LIMIT = 2;
    private static int DIO_WEDGE_UP_LIMIT = 3;
    private static int DIO_WEDGE_DOWN_LIMIT = 4;

    // Statics for Drive
    private static int TANK_DRIVE = 1;
    private static int TILT_DRIVE = 2;
    private static int TWIST_DRIVE = 3;
    DriveType driveType;
    private double left = 0;
    private double right = 0;
    int counter = 0;
    AutoScenario autoScenario;
    boolean autoMode = false;

    // Statics for Shooter Wheel Speed
    private static double PERCENT_100 = 1.0;
    private static double PERCENT_69 = 0.69;
    private static double PERCENT_40 = 0.43;
    private static double PERCENT_0 = 0;

    private static final int SHOOT_ONLY = 0;
    private static final int TIP_SHOOT = 1;
    private static final int SHOOT_TIP = 2;

    DriverStationLCD dslcd;

    DriveTrain driveTrain;

    Lift mrLift;
    Shooter mrShooter;
    Wedge mrWedge;
    BallPickup mrPickup;
    Relay mrRelay;
    AxisCamera camera;
//    AxisCamera ballCam;
    Camera spyCam;
    
    private DigitalInput wedgeUpLimit;
    private DigitalInput wedgeDownLimit;

    private SendableChooser autoScenarioChooser;
    private SendableChooser controlChooser;

    private Joystick driveStick1;
    private Joystick driveStick2;
    private Joystick gamepad;  // always the gamepad even with one joystick

//    SendableGyro mrGyro;

    int massX = 0;
    int lastMassX = 0;
    int camScan = 0;
    int jumpCounter = 0;
    int motorDir =0;
    int wantedTarget = 2;
    int lastWay = 0;
    double distance = 0.0;
    double motorSpeed = 0.1;
    final double defaultMotorSpeed = 0.1;//value for
    boolean isJumping = false;
    boolean scanBoolean = false;
    boolean hasStartedTracking = false;
    private MrWatchDog cuddles;


    NetworkTable prefsTable;

//    AutoSpeed autoSpeed;
    public String shootSpeed = "100";
    public String shootDelay = "2";

    int autoOrder[] = new int[4];
    double autoDuration[] = new double[4];
    int autoTask = 0;
    double startTime = 0.0;
    double driveSpeed = 0.3;
    boolean firstTimeShooting = true;
    boolean firstTimeDriving = true;
    static final double DURATION_BEFORE_LIFT = 3.0;//was 2 change for more
    static final double DRIVE_DURATION = 3.0;
    static final double SHOOT_DURATION = 4.0;// was 8 changed for longer speed time
    static final int DO_NOTHING = 0;
    static final int SHOOT = 1;
    static final int DRIVE = 2;

    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        NetworkTable.initialize();
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2012");

        //a new watchdog is born as a mortal
        cuddles = new MrWatchDog();

        // Limit switches
        wedgeUpLimit = new DigitalInput(DIO_WEDGE_UP_LIMIT);
        wedgeDownLimit = new DigitalInput(DIO_WEDGE_DOWN_LIMIT);

        // CAN IDs  left: 2,4  right: 3,5
        driveTrain = new DriveTrain(JAG_LEFT_A, JAG_RIGHT_A, JAG_LEFT_B, JAG_RIGHT_B);

        // CAN IDs shooter wheels: 6,7  turret: 8
        mrShooter = new Shooter(JAG_SHOOTER_WHEELS_A, JAG_SHOOTER_WHEELS_B, JAG_TURRET);

        // Victors - PWM slots on Digital Sidecar
        mrWedge = new Wedge(VIC_WEDGE, wedgeUpLimit, wedgeDownLimit);
        mrLift = new Lift(VIC_LIFT);
        mrPickup = new BallPickup(VIC_BALLPICKUP);


        // UI devices
        driveStick1 = new Joystick(1);
        driveStick2 = new Joystick(2);
        try {
            gamepad = new Joystick(3);
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
//        mrGyro = new SendableGyro(1);
//        mrGyro.reset();

        spyCam = new Camera();


        //adds the drive options to the sendable chooser, with TankDrive as the default
        controlChooser = new SendableChooser();
        controlChooser.addDefault("Default Tank Drive", new DriveType(TANK_DRIVE));
        controlChooser.addObject("X-Y Tilt Drive", new DriveType(TILT_DRIVE));
        controlChooser.addObject("Twist Stick Drive", new DriveType(TWIST_DRIVE));
        SmartDashboard.putData("Drive Chooser", controlChooser);

        //adds the speed options to the sendable chooser, with 66% as the default
        autoScenarioChooser = new SendableChooser();
        autoScenarioChooser.addDefault("Default Shoot Only", new AutoScenario(SHOOT_ONLY));
        autoScenarioChooser.addObject("Tip then Shoot", new AutoScenario(TIP_SHOOT));
        autoScenarioChooser.addObject("Shoot then Tip", new AutoScenario(SHOOT_TIP));
        SmartDashboard.putData("Autonomous Scenario", autoScenarioChooser);

        prefsTable = NetworkTable.getTable("Preferences");
        prefsTable.putString("Shooter Speed", shootSpeed);
        prefsTable.putString("Shooter Delay", shootDelay);

        //adds the option of camera alignment, with manual control as the default

//        double angle = roundTwoPlaces(mrGyro.getAngle());
        //SmartDashboard.putDouble("Gyro", angle);
    }

    public void autonomousInit()
    {
        autoTask = 0;
        startTime = Timer.getFPGATimestamp();
        shootSpeed = prefsTable.getString("Shooter Speed", "100");
        shootDelay = prefsTable.getString("Shooter Delay", "2");
        autoScenario = (AutoScenario) autoScenarioChooser.getSelected();
        dslcd.println(DriverStationLCD.Line.kUser2, 1, "Speed: " + shootSpeed);
        System.out.println("Shooting speed: " + shootSpeed);
        switch(autoScenario.getScenario())
        {
            case SHOOT_ONLY:
                //assigned to the auto array
                autoOrder = new int[]{ DO_NOTHING, SHOOT, DO_NOTHING, DO_NOTHING};
                autoDuration = new double[]{ Double.parseDouble(shootDelay),SHOOT_DURATION,0.0,0.0};
                break;
            case TIP_SHOOT:
                autoOrder = new int[]{ DRIVE, DO_NOTHING, SHOOT, DO_NOTHING};
                autoDuration = new double[]{ DRIVE_DURATION,Double.parseDouble(shootDelay),SHOOT_DURATION,0.0};
                break;
            case SHOOT_TIP:
                autoOrder = new int[]{ DO_NOTHING, SHOOT, DRIVE, DO_NOTHING};
                autoDuration = new double[]{Double.parseDouble(shootDelay),SHOOT_DURATION,DRIVE_DURATION,0.0};
                break;
        }
    }

    /**
     * This method is called periodically when the robot is in autonomous
     */
    public void autonomousPeriodic() {
        //implement the camera tracking code in here before you shoot

        //prepare for the shot (line up the drive train)

        // Start the shooter wheels at 66% power
//        mrShooter.wheelsOn();

        autoSenario(autoTask);

        // delay for wheel spinup
        System.out.println("You choose: " + autoScenario.getScenario() + " With " + shootSpeed + " shoot speed and "  + shootDelay + "  delay");
        cuddles.feed();
    }
    public void autoSenario(int q)
    {
        if(autoOrder[q] == 0)
        {
         //   Timer.delay(anomDelay[q]);
            System.out.println("Do nothing: " + Timer.getFPGATimestamp());
        }
        else if(autoOrder[q] == 1)
        {
            System.out.println("Shooting: " + Timer.getFPGATimestamp());
            autoShoot();
        }
        else if(autoOrder[q] == 2)
        {
            autoDrive();
            System.out.println("Driving: " + Timer.getFPGATimestamp());
        }

        if(hasTimePassed(autoDuration[q]) && autoTask < 3)
        {
            autoTask ++;
            firstTimeShooting = true;
            driveTrain.setSpeed(0, 0);
            mrLift.reset();
            mrShooter.wheelsOff();
            System.out.println("New task at : "  + autoTask);
        }
        if(q == 3)
        {
            System.out.println("------------DONE----------------------");
        }
        else
        {
            System.out.println("Q: " + q + " delay: " + autoDuration[q]);
        }
    }
    public void autoShoot()
    {
        //TODO: Stop motor and feed
        double speed = Double.parseDouble(shootSpeed)/100.0;
        System.out.println("speed: " + speed + " raw: " + Integer.parseInt(shootSpeed));
        mrShooter.setWheelSpeed(speed);
        if(!firstTimeShooting)
        {
             mrLift.up();
        }
        else
        {
            if(hasTimePassed(DURATION_BEFORE_LIFT))
            {
                firstTimeShooting = false;
            }
        }

    }
    public void autoDrive()
    {
        if(firstTimeDriving)
        {
            mrWedge.down();
            if(hasTimePassed(1.0))
            {
                firstTimeDriving = false;
            }
        }
        else
        {
            //TODO: PUT DRIVE HERE
            driveTrain.setSpeed(driveSpeed, -driveSpeed);// (left, right);
            System.out.println("---DRIVING@@@@@@@@@@@@@@@@@@@@@@@@");
        }
    }
    public boolean hasTimePassed(double timeWanted)
    {
        //^Says it all :Q//ps <Q face
        System.out.println("timePAssed: " + (Timer.getFPGATimestamp() - startTime) + " wanted: " + timeWanted);
        if(Timer.getFPGATimestamp() - startTime >= timeWanted)
        {
            startTime = Timer.getFPGATimestamp();
            return true;
        }
        else
        {
            return false;
        }
    }
    public void teleopInit()
    {
        driveType = (DriveType) controlChooser.getSelected();
        autoMode = false;
    }

    /**
     * This method is called periodically when the robot is in teleop
     */
    public void teleopPeriodic() {

        // Move the robot
        moveRobot();
        //SO WE CAN HAVE VALUES BEFORE MOVING BECAUAE LOGIC!
        camera();

        // Adjust Turret
        aimTurret();

        // Set the shooter wheel speed and start the wheels
        setShooter();

        // Pickup or eject balls
        pickupBalls();

        // Lift/feed balls into shooter
        feedBalls();

        //add automatic code here on the side to line up the robot to shoot

        //put wedge code here.
        moveWedge();

//        SmartDashboard.putDouble("Gyro", roundTwoPlaces(mrGyro.getAngle()));
        updateDashboard();

        dslcd.updateLCD();
        cuddles.feed();
    }

    public void moveRobot() {
        if (driveType.getDriveType() == TANK_DRIVE) {
            left = roundDirectionNum(driveStick1.getY(), 1.0);
            right = roundDirectionNum(-driveStick2.getY(), 1.0);
        } else if (driveType.getDriveType() == TILT_DRIVE) {
            left = roundDirectionNum(driveStick1.getY() - driveStick1.getX(), 1.0);
            right = roundDirectionNum(-driveStick1.getY() - driveStick1.getX(), 1.0);
            if((driveStick1.getY() < 0 && driveStick2.getY() > 0) || (driveStick1.getY() > 0 && driveStick2.getY() < 0)) {
                left = 0;
                right = 0;
            }
        } else if (driveType.getDriveType() == TWIST_DRIVE) {
            left = driveStick1.getY() + driveStick1.getTwist();
            right = driveStick1.getY() - driveStick1.getTwist();
        }
        if(driveStick1.getRawButton(3) && driveStick2.getRawButton(3))
        {
            left *= 0.5;
            right *= 0.5;
        }

        //alpha jump back code
        if(driveStick1.getRawButton(2))
        {
            //T - 10. 9. 8. 7. 6. 5. 4. 3. 2. 1. 0. Houston we have liftof
            isJumping = true;
        }

        //gravity?
        if(isJumping)
        {
            if(jumpCounter >= 2)
            {
                jumpCounter = 0;
                isJumping = false;
            }
            
            //sorry no explotions folks :(
            if(left/Math.abs(left) != right/Math.abs(right))
            {
                left = -left;
                right = -right;
                jumpCounter++;
            }
        }
        left = roundDirectionNum(left, 1.0);
        right = roundDirectionNum(right, 1.0);
        dslcd.println(DriverStationLCD.Line.kUser3, 1, "L: " + left);
        dslcd.println(DriverStationLCD.Line.kUser4, 1, "R: " + right);
        driveTrain.setSpeed(left, right);
    }

    public void setShooter() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }
        if (gamepad.getRawButton(1)) {
            mrShooter.setWheelSpeed(PERCENT_0);
        } else if (gamepad.getRawButton(2)) {
            mrShooter.setWheelSpeed(PERCENT_40);
        } else if (gamepad.getRawButton(3)) {
            mrShooter.setWheelSpeed(PERCENT_69);
            System.out.println("66 VALUE: " + PERCENT_69);
        } else if (gamepad.getRawButton(4)) {
            mrShooter.setWheelSpeed(PERCENT_100);
        }
        try
        {
            dslcd.println(DriverStationLCD.Line.kUser6, 1,"(RPM): " +  mrShooter.wheels2.getP());
        }
        catch(CANTimeoutException e)
        {
            dslcd.println(DriverStationLCD.Line.kUser6, 1, "CANTimeoutException!: " +e);
        }
    }

    /*
     *   Not used at this time...
     */
    public void setWheelMode() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }
    }

    /**
     * Read digital (left) joystick on gamepad (X)
     */
    public void aimTurret() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE)
        {
            gamepad = driveStick2;
        }
        
        //put this here instead of camera because it makes more sence
        autoMode = gamepad.getRawButton(9);
        if(gamepad.getRawButton(9))
        {
            if(!hasStartedTracking)
            {
                hasStartedTracking = true;
                motorSpeed = defaultMotorSpeed;
            }
        }
        else
        {
            hasStartedTracking = false;
        }
        if(gamepad.getRawButton(10))
        {
            //made this little number to find an image
            //because auto track only works when has image
            //so this is used to find image might merge the 
            //two latter.....
            System.out.println("MassX: " + massX);
            if(massX <= 0)
            {
                if(camScan >= 10)
                {
                    scanBoolean = !scanBoolean;
                    camScan = 0;
                }
                if(scanBoolean)
                {
                    System.out.println("Are you still there?");
                    mrShooter.turretCounterClockwise(mrShooter.TURRET_SPEED);
                }
                else
                {
                    System.out.println("Searching...Searching....");
                    mrShooter.turretClockwise(mrShooter.TURRET_SPEED);
                }
                System.out.println("C: " + camScan);
                camScan++;
            }
        }
        //if is in auto mode lets the user move the turet by own free will
        //however if autoMode is true then the user
        // is controlled by the over mind to mindlessly
        //move tword green rectangles and is helpless
        //before its wrath or untill they press 9 again
        if(!autoMode)
        {
            dslcd.println(DriverStationLCD.Line.kUser5, 1,"MANEULE READ!");
            double leftGamepadStick = gamepad.getAxis(Joystick.AxisType.kX)*-1;
            //System.out.println("LeftStrick Dir: " + leftGamepadStick);
            if (leftGamepadStick > 0.1)
            {
                mrShooter.turretCounterClockwise(mrShooter.TURRET_SPEED);
            }
            else if(leftGamepadStick < -0.1)
            {
                mrShooter.turretClockwise(mrShooter.TURRET_SPEED);
            }
            else
            {
                mrShooter.turretStop();
            }
        }
        else
        {
            motorDir = spyCam.getMotorDirection(spyCam.getPreciseRecPos(3));
//            double moverMass = Math.min(Math.abs(((massX -160.0)/160.0) + 0.05),1.0);
//            double moverMass = Math.abs(massX-160/160);
            System.out.println("MS: " + motorSpeed + " motorDir: "+ motorDir + " lastWay: " + lastWay + " massX: " + spyCam.getTargetCenter(spyCam.getPreciseRecPos(3)));
            if(lastWay !=motorDir)
            {
                //We dont want to be the same speed we w nt so e deca .   ..
                System.out.println("Changed Direction!");
                motorSpeed -= 0.02;
            }
            dslcd.println(DriverStationLCD.Line.kUser5, 1,"Motor turn: " + Double.toString(motorSpeed));
            System.out.println("Dir: " + motorDir);

            if(motorDir == -1)
            {
                //dslcd.println(DriverStationLCD.Line.kUser5, 1,"LEFT");
                mrShooter.turretClockwise(motorSpeed);
                System.out.println("MOVING RIGHT!");
            }
            else if(motorDir == 1)
            {
                //dslcd.println(DriverStationLCD.Line.kUser5, 1, "RIGHT");
                mrShooter.turretCounterClockwise(motorSpeed);
                System.out.println("MOVING LEFT!");
            }
            else if(motorDir == 0)
            {
                dslcd.println(DriverStationLCD.Line.kMain6, 1,"CENTERED");
                mrShooter.turretStop();
            }
        }
        SmartDashboard.putBoolean("Is AutoMode: ", autoMode);
        lastWay = motorDir;
    }

    public void pickupBalls() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if (gamepad.getRawButton(5))
        {
            mrPickup.setSpeed(-mrPickup.pickUpSpeed);
        } 
        else if (gamepad.getRawButton(7) && !gamepad.getRawButton(5))
        {
            mrPickup.setSpeed(mrPickup.pickUpSpeed);
        }
        else
        {
            mrPickup.setSpeed(0.0);
        }
    }

    public void feedBalls()
    {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if (gamepad.getRawButton(6))
        {
            mrLift.up();
        }
        else if (gamepad.getRawButton(8) && !gamepad.getRawButton(6))
        {
            mrLift.down();
        }
        else
        {
            mrLift.reset();
        }
    }

    public void moveWedge() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if (driveStick2.getRawButton(1) && !driveStick1.getRawButton(1))
        {
            mrWedge.up();
        } 
        else if (driveStick1.getRawButton(1) && !driveStick2.getRawButton(1))
        {
            mrWedge.down();
        }
        else
        {
            mrWedge.reset();
        }
    }

    public void updateDashboard()
    {
        if (driveType.getDriveType() == TANK_DRIVE)
        {
            dslcd.println(DriverStationLCD.Line.kMain6, 1, "Output to left side: " + driveStick1.getY());
            dslcd.println(DriverStationLCD.Line.kUser2, 1, "Output to right side: " + driveStick2.getY());
        } 
        else if (driveType.getDriveType() == TILT_DRIVE)
        {
            dslcd.println(DriverStationLCD.Line.kMain6, 1,
                    "Output to left side: " + driveStick1.getX());
            dslcd.println(DriverStationLCD.Line.kUser2, 1,
                    "Output to right side: " + (driveStick1.getX()));
        }
        else if (driveType.getDriveType() == TWIST_DRIVE)
        {
            dslcd.println(DriverStationLCD.Line.kMain6, 1,
                    "Output to left side: " + driveStick1.getX() + driveStick1.getTwist());
            dslcd.println(DriverStationLCD.Line.kUser2, 1,
                    "Output to right side: " + (driveStick1.getX() - driveStick1.getTwist()));
        }
        dslcd.println(DriverStationLCD.Line.kUser6, 1, "WDS: " + String.valueOf(cuddles.isAlive()));
        dslcd.updateLCD();
    }

    //@TODO: make dis work
    public void camera()
    {
        camera = AxisCamera.getInstance();
        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        camera.writeBrightness(50);
//        camera.writeMaxFPS(10);
//        ballCam = AxisCamera.getInstance("10.5.78.12");
//        ballCam.writeCompression(100);
//        ballCam.writeResolution(AxisCamera.ResolutionT.k160x120);
////        ballCam.writeMaxFPS(10);
        if(camera.freshImage())
        {
            try
            {
                lastMassX = massX;
//                motorDir =spyCam.getMotorDirection(spyCam.getPreciseRecPos(wantedTarget));
                distance = spyCam.getDistance(spyCam.getPreciseRecPos(3));
                 massX = spyCam.calImage(camera.getImage(), spyCam.getPreciseRecPos(3));
            }
            catch (NIVisionException ex)
            {
                ex.printStackTrace();
            }
            catch(AxisCameraException  e)
            {
                e.printStackTrace();
            }
        }
        else
        {
            massX = 0;
        }
        if(lastMassX == massX)
        {
            massX = 0;
        }
        SmartDashboard.putInt("dataTest", spyCam.getTargetCenter(3));
//        SmartDashboard.putInt("Distance", (int)distance);
//        SmartDashboard.putDouble("MotorDirection", motorDir);
////        dslcd.println(DriverStationLCD.Line.kUser5, 1,"");
//        SmartDashboard.putBoolean("Has Image: ", spyCam.hasImage);

    }

    /**
     * Used by SmartDashboard to record Drive Type selected by User
     */
    public class DriveType {

        public int driveType;

        public DriveType(int dType) {
            driveType = dType;
        }

        public int getDriveType() {
            return driveType;
        }
    }

    /**
     * Used by SmartDashboard to capture Autonomous Shooter Wheel Speed set by User
     */
    public class AutoScenario
    {

        public int autoScenario;

        public AutoScenario(int spd)
        {
            autoScenario = spd;
        }

        public int getScenario()
        {
            return autoScenario;
        }
    }

    public double roundDirectionNum(double num, double max)
    {
        double temp = num;
        if(num > max)
        {
            temp = max;
        }
        else if(num < -max)
        {
            temp = -max;
        }
        return temp;
    }

    public double roundTwoPlaces(double dbl) {
        return (double) (int) ((dbl + 0.005) * 100.0) / 100.0;
    }
}
