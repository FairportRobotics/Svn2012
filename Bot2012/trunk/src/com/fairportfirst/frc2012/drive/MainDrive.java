package com.fairportfirst.frc2012.drive;

//imports of local packages
import com.fairportfirst.frc2012.structures.*;
import com.fairportfirst.frc2012.vision.Camera;

//wiplibj imports
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
public class MainDrive extends IterativeRobot
{

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

    AutoSpeed autoSpeed;
    boolean autoMode = false;

    // Statics for Shooter Wheel Speed
    private static double PERCENT_100 = 1;
    private static double PERCENT_66 = 2/3.0;
    private static double PERCENT_45 = 1/2.25;
    private static double PERCENT_0 = 0;

    private static boolean LIFTMANUAL;

    //really important
    DriverStationLCD dslcd;

    //local instance declarations
    DriveTrain driveTrain;
    Lift mrLift;
    Shooter mrShooter;
    Wedge mrWedge;
    BallPickup mrPickup;

    Relay mrRelay;
    AxisCamera camera;
    Camera spyCam;
    
//    private DigitalInput turretCwLimit;
//    private DigitalInput turretCcwLimit;
    //limit switches for the wedge
    private DigitalInput wedgeUpLimit;
    private DigitalInput wedgeDownLimit;

    //Sendable choosers for the smart dashboard
    private SendableChooser autoSpeedChooser;
    private SendableChooser controlChooser;

    private Joystick driveStick1;
    private Joystick driveStick2;
    private Joystick gamepad;  // always the gamepad even with one joystick

    //SendableGyro mrGyro;

    
    int massX = 0;//the x coordinate of the center of the current target
    int lastMassX = 0;//the x coordinate of the center of the previous target
    int motorDir = 0;//the motor direction needed for panning
    int camScan = 0;//the timer for panning
    int jumpCounter = 0;//a time counter for the jumpback function
    int wantedTarget = 0;//the number ID of the target
    double distanceTarget = 0.0;//the distance to the target in inches
    boolean isJumping = false;//if the robot is jumping or not
    boolean scanBoolean = false;//determines the spin direction of the turret

    private MrWatchDog cuddles;//the super important watchdog


    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
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


        mrRelay = new Relay(1, 1);

        // UI devices
        driveStick1 = new Joystick(1);
        driveStick2 = new Joystick(2);

        //tries to asign the gamepad
        try
        {
            gamepad = new Joystick(3);
        } 
        catch (NullPointerException e)
        {
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
        autoSpeedChooser = new SendableChooser();
        autoSpeedChooser.addDefault("Default 66%", new AutoSpeed(PERCENT_66));
        autoSpeedChooser.addObject("100%", new AutoSpeed(PERCENT_100));
        autoSpeedChooser.addObject("45%", new AutoSpeed(PERCENT_45));
        autoSpeedChooser.addObject("0%", new AutoSpeed(PERCENT_0));
        SmartDashboard.putData("Autonomous Speed", autoSpeedChooser);

        //adds the option of camera alignment, with manual control as the default

//        double angle = roundTwoPlaces(mrGyro.getAngle());
        //SmartDashboard.putDouble("Gyro", angle);

        mrRelay.set(Relay.Value.kForward);

        
    }

    /**
     * This method is called when the Autonomous period begins
     */
    public void autonomousInit()
    {
        autoSpeed = (AutoSpeed) autoSpeedChooser.getSelected();
    }

    /**
     * This method is called periodically when the robot is in autonomous
     */
    public void autonomousPeriodic()
    {
        //implement the camera tracking code in here before you shoot

        //prepare for the shot (line up the drive train)

        // Start the shooter wheels at 66% power
        mrShooter.setWheelSpeed(autoSpeed.getSpeed());
//        mrShooter.wheelsOn();

        // delay for wheel spinup
        Timer.delay(3.0);
        mrLift.up();
        cuddles.feed();
    }

    /**
     * This method is called when the TeleOperated period begins
     *
     */
    public void teleopInit()
    {
        driveType = (DriveType) controlChooser.getSelected();
    }

    /**
     * This method is called periodically when the robot is in teleop
     */
    public void teleopPeriodic()
    {

        // Move the robot
        moveRobot();

       //SO WE CAN HAVE VALUES BEFORE MOVING BECAUAE LOGIC!
        camera();

        // Adjust Turret
        aimTurret();
        // Set the shooter wheel speed and start the wheels
        setShooter();

        // Adjust Turret

        //did this in teleopContinuous ^
//        aimTurret();

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

    /**
     * moves the robot with input values from the Joysticks
     */
    public void moveRobot()
    {
        if (driveType.getDriveType() == TANK_DRIVE)
        {
            left = roundDirectionNum(driveStick1.getY(), 1.0);
            right = roundDirectionNum(-driveStick2.getY(), 1.0);
//            if((driveStick1.getX() < 0 && driveStick2.getX() > 0) || (driveStick1.getX() > 0 && driveStick2.getX() < 0)) {
//                left = 0;
//                right = 0;
//            }
        } 
        else if (driveType.getDriveType() == TILT_DRIVE)
        {
            left = roundDirectionNum(driveStick1.getY() - driveStick1.getX(), 1.0);
            right = roundDirectionNum(-driveStick1.getY() - driveStick1.getX(), 1.0);
            if((driveStick1.getY() < 0 && driveStick2.getY() > 0) || (driveStick1.getY() > 0 && driveStick2.getY() < 0))
            {
                left = 0;
                right = 0;
            }
        } 
        else if (driveType.getDriveType() == TWIST_DRIVE)
        {
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

    /**
     * Sets the speed of the flywheels in the shooter
     */
    public void setShooter()
    {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE)
        {
            gamepad = driveStick2;
        }
        if(!autoMode)
        {
            if (gamepad.getRawButton(1))
            {
                mrShooter.setWheelSpeed(PERCENT_0);
            }
            else if (gamepad.getRawButton(2))
            {
                mrShooter.setWheelSpeed(PERCENT_45);
            }
            else if (gamepad.getRawButton(3))
            {
                mrShooter.setWheelSpeed(PERCENT_66);
            }
            else if (gamepad.getRawButton(4))
            {
                mrShooter.setWheelSpeed(PERCENT_100);
            }
        }
        //auto mode :D
        else
        {
            //Sets the disired target for the user if auto mode is enabled
            // these are the presice values
            // 0 - top rec
            // 1 - left rec
            // 2 - right rec
            // 3 - bottom rec
            if(gamepad.getRawButton(4))
            {
                wantedTarget = 0;
            }
            else if(gamepad.getRawButton(1))
            {
                wantedTarget = 1;
            }
            else if(gamepad.getRawButton(3))
            {
                wantedTarget = 2;
            }
            else if(gamepad.getRawButton(2))
            {
                wantedTarget = 3;
            }

            //lets the user if is in auto mode to choose the speed of the fireing
            //wheels with the left gamepad joystiick in the Y-axis
            //currently untested
            double leftPadMove = roundTwoPlaces(gamepad.getAxis(Joystick.AxisType.kY));
            System.out.println("GamePad Left-Y: " + leftPadMove);
            if(leftPadMove > 0)
            {
                mrShooter.setWheelSpeed(leftPadMove);
            }
            else if(gamepad.getRawButton(7))
            {
                mrShooter.setWheelSpeed(spyCam.distanceToMotor(distanceTarget));
            }
        }

        //tested RPM from the encoders on the shooter motors
        //curently does not work could be hardware prolems
        try
        {
            //mrShooter.turret.getPosition();
            
            dslcd.println(DriverStationLCD.Line.kUser6, 1,"(RPM): " +  mrShooter.wheels2.getSpeed());
        }
        catch(CANTimeoutException e)
        {
            dslcd.println(DriverStationLCD.Line.kUser6, 1, "CANTimeoutException!: " +e);
        }
    }

    /*
     *   Not used at this time...
     */
    public void setWheelMode()
    {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE)
        {
            gamepad = driveStick2;
        }

//        if(mrJoystick3.getAxisChannel(Joystick.AxisType.kNumAxis)) {
//
//        }
    }

    /**
     * Turns the turret to aim it with inputs from the joysticks. Also has an
     * automatic function for the camera panning
     */
    public void aimTurret()
    {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE)
        {
            gamepad = driveStick2;
        }
        
        //put this here instead of camera because it makes more sence
        if(gamepad.getRawButton(9))
        {
            autoMode = !autoMode;
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
            //extras space needed for printout to clear the clogaged by labview
            dslcd.println(DriverStationLCD.Line.kUser5, 1,"MANEULE READ!             ");
            double leftGamepadStick = gamepad.getAxis(Joystick.AxisType.kX);
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
        else if(gamepad.getRawButton(5))
        {
            double moverMass = Math.abs(((massX -160.0)/160.0) + 0.05);
            dslcd.println(DriverStationLCD.Line.kUser5, 1,Double.toString(moverMass));
            System.out.println("DIr: " + moverMass);
            if(motorDir == 1)
            {
                //dslcd.println(DriverStationLCD.Line.kUser5, 1,"LEFT");
                mrShooter.turretClockwise(moverMass);
            }
            else if(motorDir == -1)
            {
                //dslcd.println(DriverStationLCD.Line.kUser5, 1, "RIGHT");
                mrShooter.turretCounterClockwise(moverMass);
            }
            else if(motorDir == 0)
            {
                dslcd.println(DriverStationLCD.Line.kMain6, 1,"CENTERED");
                mrShooter.turretStop();
            }
        }
    }

    /**
     * the method that runs the intake rollers
     */
    public void pickupBalls()
    {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }
        if(!autoMode)
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
    /**
     * the method that controls the lift of the robot
     */
    public void feedBalls()
    {
            // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
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

    /**
     * the method that runs the wedge to put down the bridge
     */
    public void moveWedge()
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

    /**
     * updates the data to the dashboard (duh)
     */
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
    /**
     * gets an image from the camera
     */
    public void camera()
    {
        camera = AxisCamera.getInstance();
        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        camera.writeBrightness(50);
        if(camera.freshImage())
        {
            try
            {
                lastMassX = massX;
                massX = spyCam.calImage(camera.getImage(), spyCam.getPreciseRecPos(wantedTarget));
                motorDir = spyCam.getMotorDirection(spyCam.getPreciseRecPos(wantedTarget));
                distanceTarget = spyCam.getDistance(spyCam.getPreciseRecPos(wantedTarget));
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
        SmartDashboard.putInt("dataTest", massX);
        SmartDashboard.putInt("Distance", (int)distanceTarget);
        SmartDashboard.putDouble("MotorDirection", motorDir);
//        dslcd.println(DriverStationLCD.Line.kUser5, 1,"");
        SmartDashboard.putBoolean("Has Image: ", spyCam.hasImage);
    }

    /**
     * Used by SmartDashboard to record Drive Type selected by User
     */
    public class DriveType
    {

        public int driveType;

        public DriveType(int dType)
        {
            driveType = dType;
        }

        public int getDriveType()
        {
            return driveType;
        }
    }

    /**
     * Used by SmartDashboard to capture Autonomous Shooter Wheel Speed set by User
     */
    public class AutoSpeed
    {

        public double autoSpeed;

        public AutoSpeed(double spd)
        {
            autoSpeed = spd;
        }

        public double getSpeed()
        {
            return autoSpeed;
        }
    }

    /**
     * rounds the input number to
     * @param num the number to be rounded
     * @param max the boundary value
     * @return
     */
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

    /**
     * rounds the input to two decimal places
     * @param dbl the input value
     * @return the input value rounded to two decimal places
     */
    public double roundTwoPlaces(double dbl)
    {
        return (double) (int) ((dbl + 0.005) * 100.0) / 100.0;
    }
}
