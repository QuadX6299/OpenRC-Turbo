package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HerculesLibraries.Vision.NewBitMap;
import org.firstinspires.ftc.teamcode.Robot.Bhorn;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Grabber;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.lib.Constraints.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.lib.Constraints.TankKinematics;
import org.firstinspires.ftc.teamcode.lib.Coords.Position;
import org.firstinspires.ftc.teamcode.lib.Coords.Waypoint;
import org.firstinspires.ftc.teamcode.lib.Path.PathFollower;
import org.firstinspires.ftc.teamcode.lib.Path.Paths;

import java.util.List;

@Autonomous(name = "BlueAuto", group = "Auto")

public class BlueAuto extends LinearOpMode {
    private AutoStates state;
    private PathFollower pp;
    private ElapsedTime t;
    double lastT = 0.0;

    private String skyStonePosition = "C";


//    Handler dispatcher;


    double kP = 0.001;
    double kI = 0.0;
    double kD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.init(this);
//        state = AutoStates.VISION;
//        Robot.init(this);
        t = new ElapsedTime();
//        pp = new PathFollower(Paths.getStraightLineLeft(), 10.0, new TankKinematics(DriveTrain.width + 10.0), new PIDFCoefficients(.007,0.0,.0003,1/120.0,.0003),
//                5.0, this);
        t.reset();
//        dispatcher = new Handler(Looper.getMainLooper());
        Robot.reset();
        DriveTrain.initAuto(this);

        //vision = new TensorFlowDetection(this);

        while(!isStarted()){
            skyStonePosition = NewBitMap.isShriggaBlue();
            telemetry.addData("Skystone Position: ", skyStonePosition);
            telemetry.update();
        }



        waitForStart();

        t.reset();



        if (skyStonePosition.equals("L")){
            pp = new PathFollower(Paths.getStraightLineLeft(), 10.0, new TankKinematics(DriveTrain.width + 10.0), new PIDFCoefficients(.007,0.0,.0003,1/120.0,.0003),
                    5.0, this);
        } else if (skyStonePosition.equals("C")){
            pp = new PathFollower(Paths.getStraightLine(), 7.0, new TankKinematics(DriveTrain.width + 10.0), new PIDFCoefficients(.007,0.0,.0003,1/120.0,.0003),
                    5.0, this);
        } else if (skyStonePosition.equals("R")){
            pp = new PathFollower(Paths.getStraightLineRight(), 10.0, new TankKinematics(DriveTrain.width + 10.0), new PIDFCoefficients(.007,0.0,.0003,1/120.0,.0003),
                    5.0, this);
        }
//
        while (!pp.isDone()) {
            lastT = t.seconds() - lastT;
            Waypoint rloc = Robot.PurePursuit(lastT);
            List<Double> powers = pp.followPath(new Position(rloc.getX(), rloc.getY(), rloc.getDdx()), rloc.getDx(), rloc.getDy(), lastT);
            DriveTrain.setPower(powers.get(0), powers.get(1));
        }

        Bhorn.toggle();
        Thread.sleep(500);
        DriveTrain.setPower(-.25, -.25);
        Thread.sleep(300);
        DriveTrain.setPower(.2, .2);
        Intake.power(1.0);
        Thread.sleep(300);
        Bhorn.toggle();
        DriveTrain.stopMotors();
        Thread.sleep(1000);
        Grabber.setPosition(Grabber.POSITIONS.PUSHTHROUGH);
        Intake.stopIntake();
        /*
        DriveTrain.turnPID(.3,true, 3.0*Math.PI/2.0, 3000);
        pp.reset(Paths.getPogpursuit(), true);
        while (!pp.isDone()) {
            lastT = t.seconds() - lastT;
            Waypoint rloc = Robot.PurePursuit(lastT);
            List<Double> powers = pp.followPath(new Position(rloc.getX(), rloc.getY(), rloc.getDdx()), rloc.getDx(), rloc.getDy(), lastT);
            DriveTrain.setPower(powers.get(0), powers.get(1));
        }

         */
        //DriveTrain.turnPID(.3, false, 3*Math.PI/2.0,3000);
//        DriveTrain.setPower(-.25,-.25);
//        Thread.sleep(500);
//        DriveTrain.stopMotors();
//        FoundationHook.toggle();
//        Thread.sleep(1000);
//
        //Grabber.setPosition(Grabber.POSITIONS.PUSHTHROUGH);
////        DriveTrain.turnPID(0.0001, 0.0, 0.0, true, 90, 2000);
//
        //Thread.sleep(1000);

//        Robot.manipMachine();
//        Thread.sleep(1000);
//        Robot.manipMachine();
//        Thread.sleep(2000);
//        Robot.manipMachine();
//        Thread.sleep(2000);
//        Grabber.setPosition(Grabber.POSITIONS.DROP);
//        Thread.sleep(2000);
//        DriveTrain.turnPID(.3, false, Math.PI, 3000);
//        Thread.sleep(500);
//
//        Robot.manipMachine();
//        Thread.sleep(1000);
//
//        FoundationHook.toggle();
//        Thread.sleep(500);
//
//        DriveTrain.setPower(-0.2,-0.2);
//        Thread.sleep(1200);
//        DriveTrain.stopMotors();






    }
}