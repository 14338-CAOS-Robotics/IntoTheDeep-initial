package org.firstinspires.ftc.teamcode.Control;

import static org.firstinspires.ftc.teamcode.Control.Movement.MovementDash.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Hardware.Physical.Hardware;
import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;

import java.util.ArrayList;


public class Movement {
    @Config
    public static class MovementDash{

        //.06, 0, ,002
        public static double moveP = .04,   moveI = 0,  moveD = 0.01;
        public static double rotP = 0.75, rotI = 0, rotD = 0.05;
        public static double holdMoveP = .04, holdMoveI = 0, holdMoveD = 0.01;
        public static double FcConstant = 2000;
        public static double testingPower = .75;
        public static double teleopcirclePowScalar = 1;

    }

    public ThreeWheelOdometry odo;
    public Hardware hardware;

    //drive motor power constants
    double rightFrontPowerConstant;
    double leftFrontPowerConstant;
    double leftBackPowerConstant;
    double rightBackPowerConstant;

    public final PIDController headingPID;
    public final PIDController movementPID;
    //hold position PIDs
//    public PIDController holdHeadingPID;
    public PIDController holdMovementPID;
    private ElapsedTime runtime;

    public double t = 0;
    public double tX, tY;


    RingBuffer<Double> xValues = new RingBuffer<Double>(10, 0.00001);
    RingBuffer<Double> yValues = new RingBuffer<Double>(10, 0.00001);

    private volatile double[][] velocityRingBuffer = new double[][] {{1,0, 999999999},{2,0, 999999999},{3,0, 999999999},{4,0, 999999999},{5,0, 999999999},{6,0, 999999999},{7,0, 999999999},{8,0, 999999999},{9,0, 999999999},{10,0, 999999999} };
    private volatile int velocityRingBufferCount = 0;




    public Movement(double startX, double startY, double startRot, Hardware hardware){
        this.hardware = hardware;

        hardware.lockDrive();
        stopDrive();

        odo = new ThreeWheelOdometry(startX, startY, startRot, hardware);

        headingPID = new PIDController(rotP, rotI, rotD, 1);
        movementPID = new PIDController(moveP,moveI,moveD, 1);

//        holdHeadingPID = new PIDController(1, 0, .103, 0);
        holdMovementPID = new PIDController(holdMoveP,holdMoveI,holdMoveD, 0);

        rightBackPowerConstant = hardware.getRightBackPowerScalar();
        rightFrontPowerConstant = hardware.getRightFrontPowerScalar();
        leftBackPowerConstant = hardware.getLeftBackPowerScalar();
        leftFrontPowerConstant = hardware.getLeftFrontPowerScalar();

        runtime = new ElapsedTime();
        runtime.reset();

    }

    double prevDerivative = 0;
    public Vector2d joyStickDriveSimple(double[] driveStick, double power) {
        odo.localize();
        double[] location = odo.getLocation();

        double deltaY = Math.abs(yValues.getValue(location[1]) - location[1]);
        double deltaX = Math.abs(xValues.getValue(location[0]) - location[0]);
        if (deltaX == 0) deltaX = .00000001;


        double[] velocity = getVelocityTELE(location[0], location[1], runtime.milliseconds());
//        System.out.println(velocity[0]);
        double circlePow = 0;
        double derivative = deltaY/deltaX;
        if(velocity[0] > .05){
            circlePow = MovementDash.FcConstant * Math.pow(velocity[0], 2)
                    * ((derivative-prevDerivative)/deltaX)
                    / Math.pow(Math.hypot(1, derivative), 3);
        }
        prevDerivative = derivative;

        double rotationFactor = -Math.asin(Range.clip(circlePow / (power+.0000001), -.65, .65));
        double dX = Math.cos(rotationFactor) * driveStick[0] + Math.sin(rotationFactor) * driveStick[1];
        double dY = Math.cos(rotationFactor) * driveStick[1] - Math.sin(rotationFactor) * driveStick[0];

        return new Vector2d(dX, -dY);
    }

    //tIncrement .003 for 200cm
    public boolean followPath(Function[] path, double power, double heading, double tCutoff, double tIncrement, double endThreshold, double endPower, boolean stopAtEnd, boolean curvy){

        //updating position
        odo.localize();
        double location[] = odo.getLocation();
        //getting heading correction
        double headingCorrection = catchJump(heading, -location[2]);
        //update and get PID
        double headingPower = getHeadingPower(headingCorrection);

        //target point section of movement
        if(t >= tCutoff){
            double endX = path[0].get(1); double endY = path[1].get(1);

            //if within ending threshold (currently square)
            if(Math.abs(endX - location[0]) < endThreshold && Math.abs(endY - location[1]) < endThreshold){
                //reset path
                t = 0;
                //stop drive
                if(stopAtEnd) {
                    stopDrive();
                }
                return false;
            }
            //target point
            drive(endX - location[0], endY - location[1], location[2], endPower, headingPower);


        }else{
            //Path following section of movement

            //distance of robot to last closest point
            double dist = Math.pow(tX - location[0], 2) + Math.pow(location[1]  - tY, 2);

            //incrementing t to find new closest t
            double newT = t + tIncrement;
            double newTX = path[0].get(newT);
            double newTY = path[1].get(newT);
            double newDist = Math.pow(newTX - location[0], 2) + Math.pow(location[1]  - newTY, 2);

            //while the distance is decreasing (point is moving closer to the robot) keep incremeting t
            while(dist > newDist){
                t = newT;
                tX = newTX;
                tY = newTY;

                dist = newDist;

                newT += tIncrement;

                newTX = path[0].get(newT);
                newTY = path[1].get(newT);
                newDist = Math.pow(newTX - location[0], 2) + Math.pow(location[1]  - newTY, 2);

            }
            //newT is effectively seeking point - t is the closest to the robot and newT is one increment further - then used to draw a secant line to follow

            //finds angle between line newT-t and t-location, the sign of the value determines which side of the path the robot is on, then used for correcting PID
            double secLineAngle = Math.atan2(newTY-tY, newTX-tX);
            double pathDisplacementAngle = catchJump(Math.atan2(tY-location[1], tX-location[0]), secLineAngle);
            //correcting PID, uses distance from the path the apply a power perpendicular to the secant line
            movementPID.update(Math.sqrt(dist) * Math.signum(pathDisplacementAngle));

            if(curvy){
                double dydx = path[2].get(t);
                double d2ydx2 = path[3].get(t);

                //finds angle between secant line angle and first derivative to get the direction of centripetal acceleration
                double pathCurvatureAngle = catchJump(Math.atan(dydx), secLineAngle);

                double r = Math.pow(1+Math.pow(dydx, 2), 1.5) / Math.abs(d2ydx2) * Math.signum(pathCurvatureAngle) * Math.signum(-newTX+tX);

                double[] velocity = getVelocity(tX, tY, runtime.milliseconds());

                double circlePow = MovementDash.FcConstant * Math.pow(velocity[0], 2) / r;

                //System.out.println("Circle power is: " + circlePow + " velocity is: " + velocity[0]*100);

                //following the secant line formed by t and newT
                drive(newTX - tX, newTY - tY, location[2], power, headingPower, circlePow + movementPID.getCorrection());
            }else{
                drive(newTX - tX, newTY - tY, location[2], power, headingPower, movementPID.getCorrection());
            }

        }

        return true;
    }

    public boolean snapHeading(double heading, double headingThreshold) {
        while (Math.abs(catchJump(heading, odo.getHeading())) > headingThreshold) {
            odo.localize();
            double currentHeading = odo.getHeading();
            double headingCorrection = catchJump(currentHeading, heading);
            double headingPower = getHeadingPower(headingCorrection);
            drive(0,0,heading,0.3,headingPower);
        }
        stopDrive();
        return true;
    }


    public boolean followTankPath(Function[] path, double power, double finalHeading, double tCutoff, double tIncrement, double endThreshold, double endPower, boolean stopAtEnd, boolean curvy) {
        double heading;
        if(t >= tCutoff){
            heading = finalHeading;
        }else{
            heading = Math.atan(path[2].get(t));
        }
        return followPath(path, power, heading, tCutoff, tIncrement, endThreshold, endPower, stopAtEnd, curvy);
    }

    public boolean followPathTank(Function[] path, double power, double tCutoff, double tIncrement, boolean stopAtEnd, boolean curvy){

        //updating position
        odo.localize();
        double location[] = odo.getLocation();

        if(t > tCutoff){
            t = 0;
            if(stopAtEnd){
                stopDrive();
            }
            return false;
        }

        //target point section of movement

        //Path following section of movement

        //distance of robot to last closest point
        double dist = Math.pow(tX - location[0], 2) + Math.pow(location[1]  - tY, 2);

        //incrementing t to find new closest t
        double newT = t + tIncrement;
        double newTX = path[0].get(newT);
        double newTY = path[1].get(newT);
        double newDist = Math.pow(newTX - location[0], 2) + Math.pow(location[1]  - newTY, 2);

        //while the distance is decreasing (point is moving closer to the robot) keep incremeting t
        while(dist > newDist){
            t = newT;
            tX = newTX;
            tY = newTY;

            dist = newDist;

            newT += tIncrement;

            newTX = path[0].get(newT);
            newTY = path[1].get(newT);
            newDist = Math.pow(newTX - location[0], 2) + Math.pow(location[1]  - newTY, 2);

        }
        //newT is effectively seeking point - t is the closest to the robot and newT is one increment further - then used to draw a secant line to follow

        //finds angle between line newT-t and t-location, the sign of the value determines which side of the path the robot is on, then used for correcting PID
        double secLineAngle = Math.atan2(newTY-tY, newTX-tX);
        //getting heading correction
        double headingCorrection = catchJump(secLineAngle, -location[2]);
        //update and get PID
        double headingPower = getHeadingPower(headingCorrection);
        double pathDisplacementAngle = catchJump(Math.atan2(tY-location[1], tX-location[0]), secLineAngle);
        //correcting PID, uses distance from the path the apply a power perpendicular to the secant line
        movementPID.update(Math.sqrt(dist) * Math.signum(pathDisplacementAngle));

        if(curvy){
            double dydx = path[2].get(t);
            double d2ydx2 = path[3].get(t);

            //finds angle between secant line angle and first derivative to get the direction of centripetal acceleration
            double pathCurvatureAngle = catchJump(Math.atan(dydx), secLineAngle);

            double r = Math.pow(1+Math.pow(dydx, 2), 1.5) / Math.abs(d2ydx2) * Math.signum(pathCurvatureAngle) * Math.signum(-newTX+tX);

            double[] velocity = getVelocity(tX, tY, runtime.milliseconds());

            double circlePow = MovementDash.FcConstant * Math.pow(velocity[0], 2) / r;

            //System.out.println("Circle power is: " + circlePow + " velocity is: " + velocity[0]*100);

            //following the secant line formed by t and newT
            drive(newTX - tX, newTY - tY, location[2], power, headingPower, circlePow + movementPID.getCorrection());
        }else{
            drive(newTX - tX, newTY - tY, location[2], power, headingPower, movementPID.getCorrection());
        }



        return true;
    }

    /**
     *
     * @param absx
     * @param absy
     * @param heading
     * @param power
     * @param headingPower
     */
    public void drive(double absx, double absy, double heading, double power, double headingPower){
        double maxPower = 1 - Math.abs(headingPower);
        power = Range.clip(power, -maxPower, maxPower);

        //no clue TBH
        double xDiff = absx;
        double yDiff = absy;

        //rotate XY plane to treat it as an h drive
        double rotationFactor = -Math.PI/4 + heading;
        double dX = Math.cos(rotationFactor) * xDiff + Math.sin(rotationFactor) * yDiff;
        double dY = Math.cos(rotationFactor) * yDiff - Math.sin(rotationFactor) * xDiff;

        //scale down dX and dY to lie on the unit circle
        double DD = Math.max(Math.abs(dX), Math.abs(dY));
        double powX = dX/DD;
        double powY = dY/DD;

        powX *= power;
        powY *= power;

        //multiply by desired power since previous calculation was out of 1
        hardware.rightFront.setPower((powX-headingPower) * rightFrontPowerConstant);
        hardware.leftBack.setPower((powX+headingPower) * leftBackPowerConstant);
        hardware.leftFront.setPower((powY+headingPower) * leftFrontPowerConstant);
        hardware.rightBack.setPower((powY-headingPower) * rightBackPowerConstant);
        //back right is a bitch
    }

    public void drive(double absx, double absy, double heading, double power, double headingPower, double perpendicularPower){
        double maxPower = 1 - Math.abs(headingPower);
        power = Range.clip(power, -maxPower, maxPower);

        //Still no clue
        double xDiff = absx;
        double yDiff = absy;

        //rotate XY plane to treat as H drive. Also rotating additionally to simulate perpendicular power
        double rotationFactor = -Math.PI/4 + heading - Math.asin(Range.clip(perpendicularPower / (power+.0000001), -.65, .65));
        double dX = Math.cos(rotationFactor) * xDiff + Math.sin(rotationFactor) * yDiff;
        double dY = Math.cos(rotationFactor) * yDiff - Math.sin(rotationFactor) * xDiff;

        //scale down dX and dY to lie on the unit circle
        double DD = Math.max(Math.abs(dX), Math.abs(dY));
        double powX = dX/DD;
        double powY = dY/DD;

        powX *= power;
        powY *= power;

        //multiply by desired power since previous calculation was out of 1
        hardware.rightFront.setPower((powX-headingPower) * rightFrontPowerConstant);
        hardware.leftBack.setPower((powX+headingPower) * leftBackPowerConstant);
        hardware.leftFront.setPower((powY+headingPower) * leftFrontPowerConstant);
        hardware.rightBack.setPower((powY-headingPower) * rightBackPowerConstant);

    }

    public void holdPosition(double x, double y, double hPow, int i, int j){
//        holdHeadingPID.setConstants(MovementDash.holdHeadP, MovementDash.holdHeadI, MovementDash.holdHeadD);
        holdMovementPID.setConstants(MovementDash.holdMoveP,MovementDash.holdMoveI,MovementDash.holdMoveD);
        //updating position
        odo.localize();
        double location[] = odo.getLocation();
        //getting heading correction

        //update and get PID
        double headingPower = -hPow;
        double xDiff = x-location[0];
        double yDiff = y-location[1];

        holdMovementPID.update(Math.hypot(xDiff, yDiff));

        drive(xDiff, yDiff, location[2],holdMovementPID.getCorrection(), headingPower);
    }

    public void holdPosition(double x, double y, double h, double hPowScalar){
        //holdMovementPID.setConstants(MovementDash.holdMoveP,MovementDash.holdMoveI,MovementDash.holdMoveD);
        //updating position
        odo.localize();
        double location[] = odo.getLocation();
        //getting heading correction
        double headingCorrection = catchJump(h, -location[2]);
        //update and get PID
        double headingPower = getHeadingPower(headingCorrection) * hPowScalar;
        double xDiff = x-location[0];
        double yDiff = y-location[1];

        holdMovementPID.update(Math.hypot(xDiff, yDiff));

        drive(xDiff, yDiff, location[2],holdMovementPID.getCorrection(), headingPower);
    }
    public void holdPosition(double x, double y, double h){
        //holdMovementPID.setConstants(MovementDash.holdMoveP,MovementDash.holdMoveI,MovementDash.holdMoveD);
        //updating position
        odo.localize();
        double location[] = odo.getLocation();
        //getting heading correction
        double headingCorrection = catchJump(h, -location[2]);
        //update and get PID
        double headingPower = getHeadingPower(headingCorrection);
        double xDiff = x-location[0];
        double yDiff = y-location[1];

        holdMovementPID.update(Math.hypot(xDiff, yDiff));

        drive(xDiff, yDiff, location[2],holdMovementPID.getCorrection(), headingPower);
    }
    public double[] test = new double[]{0,0,0,0};
    //Input points of a bezier curve in the order you want them to be calculated
    public Function[] getBezierCurve(double[][] points){
        Function[] spline = new Function[4];
        int n = points.length-1;
        spline[0] = (u) -> {
            double x = 0;
            for(int i = 0; i <= n; i++){
                x += B(n, i, u) * points[i][0];
            }
            return x;
        };
        spline[1] = (u) -> {
            double y = 0;
            for(int i = 0; i <= n; i++){
                y += B(n, i, u) * points[i][1];
            }
            return y;
        };

        spline[2] = (u) -> {
            double dx = 0;
            double dy = 0;

            for(int i = 0; i <= n-1; i++){
                dx += B(n-1, i, u) * ((n) * (points[i+1][0] - points[i][0]));
            }

            if(dx == 0){
                u += .0001;

                for(int i = 0; i <= n-1; i++){
                    dx += B(n-1, i, u) * ((n) * (points[i+1][0] - points[i][0]));
                }
            }

            for (int i = 0; i <= n - 1; i++) {
                dy += B(n - 1, i, u) * ((n) * (points[i + 1][1] - points[i][1]));
            }


            return dy/dx;

        };

        spline[3] = (u) -> {
            double dx = 0, dy = 0, d2x = 0, d2y = 0;

            for(int i = 0; i <= n-1; i++){
                dx += B(n-1, i, u) * ((n) * (points[i+1][0] - points[i][0]));
            }

            if(dx == 0){
                u += .0001;
                for(int i = 0; i <= n-1; i++){
                    dx += B(n-1, i, u) * ((n) * (points[i+1][0] - points[i][0]));
                }
            }

            for(int i = 0; i <= n-1; i++){
                dy += B(n-1, i, u) * ((n) * (points[i+1][1] - points[i][1]));
            }
            for(int i = 0; i <= n-2; i++){
                d2y += B(n-2, i, u) * ((n) *(n-1) * (points[i+2][1] -2 * points[i+1][1] + points[i][1]));
            }
            for(int i = 0; i <= n-2; i++){
                d2x += B(n-2, i, u) * ((n) *(n-1) * (points[i+2][0] -2 * points[i+1][0] + points[i][0]));
            }

            return ((dx * d2y) - (d2x * dy)) / Math.pow(dx, 3);



        };

        return spline;
    }

    //takes 2 points, start and end
    public Function[] getLine(double[][] points){
        //t goes between 0 and 1, 1 being the endpoint of the graph
        double xDiff = points[1][0] - points[0][0];
        double yDiff = points[1][1] - points[0][1];
        Function[] line = new Function[4];
        line[0] = (t) -> t*xDiff+points[0][0];
        line[1] = (t) -> t*yDiff+points[0][1];
        if(xDiff==0){
            line[2] = (t) -> 999999;
        }else{
            line[2] = (t) -> yDiff/xDiff;
        }

        return line;
    }

    //takes start point, center of rotation, and fraction of circle wanted to be rotated (make sure to include direction with +/-
    public Function[] getOrbit(double[][] points){

        double[] center = points[0];
        double[] start = points[1];
        double fraction = points[2][0];
        double radius = Math.sqrt(Math.pow(center[0] - start[0], 2) + Math.pow(center[1] - start[1], 2));

        Function[] arc = new Function[2];

        arc[0] = (t) -> center[0] + Math.cos(t*2*Math.PI*fraction)*radius;
        arc[1] = (t) -> center[1] + Math.sin(t*2*Math.PI*fraction)*radius;

        return arc;
    }

    //faster
    private static double fact(int n){
        switch (n){
            case 0:
            case 1: return 1;
            case 2: return 2;
            case 3: return 6;
            case 4: return 24;
            case 5: return 120;
            case 6: return 720;
            default: return 0;
        }
    }

    private double B(int n, int i, double u){ return fact(n)/(fact(i) * fact(n-i)) * (double)Math.pow(u, i) * (double)Math.pow(1-u, n-i); }

    public void stopDrive(){
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.leftFront.setPower(0);
        hardware.rightBack.setPower(0);
    }

    public interface Function{ double get(double t);}

    public double catchJump(double to, double from){
        //makes sure that difference is within the range of -pi to pi instead of out of 2 pi
        double diff = to - from;
        while(diff > Math.PI){
            diff -= Math.PI * 2;
        }
        while(diff < -Math.PI){
            diff += Math.PI * 2;
        }
        return diff;
    }

    public double getHeadingPower(double headingError){
        headingPID.update(headingError);
        return headingPID.getCorrection();

    }

    public double[] getVelocity(double x, double y, double time){
        int current = velocityRingBufferCount % velocityRingBuffer.length;
        int last = (velocityRingBufferCount+1) % velocityRingBuffer.length;

        velocityRingBuffer[current][0] = x;
        velocityRingBuffer[current][1] = y;
        velocityRingBuffer[current][2] = time;

        double dX = x-velocityRingBuffer[last][0];
        double dY = y-velocityRingBuffer[last][1];
        double dT = time-velocityRingBuffer[last][2];

        double dDistance = Math.hypot(dX, dY);
        double dTime = dT;

        double speed = dDistance/dTime;
        double slope;
        if(dX == 0){
            slope = 9999999;
        }else{
            slope = dY/dX;
        }

        velocityRingBufferCount++;

        return new double[]{speed, slope};
    }

    public double[] getVelocityTELE (double x, double y, double time){
        int current = velocityRingBufferCount % 3;
        int last = (velocityRingBufferCount+1) % 3;

        velocityRingBuffer[current][0] = x;
        velocityRingBuffer[current][1] = y;
        velocityRingBuffer[current][2] = time;

        double dX = x-velocityRingBuffer[last][0];
        double dY = y-velocityRingBuffer[last][1];
        double dT = time-velocityRingBuffer[last][2];

        double dDistance = Math.hypot(dX, dY);
        double dTime = dT;

        double speed = dDistance/dTime;
        double slope;
        if(dX == 0){
            slope = 9999999;
        }else{
            slope = dY/dX;
        }

        velocityRingBufferCount++;

        return new double[]{speed, slope};
    }

    public void setHeadingPID() {
        headingPID.setConstants(rotP, rotI, rotD);
    }
    public void setMovementPID() {
        movementPID.setConstants(moveP, moveI, moveD);
    }
}
