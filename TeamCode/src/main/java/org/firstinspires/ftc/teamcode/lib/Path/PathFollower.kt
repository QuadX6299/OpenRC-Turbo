package org.firstinspires.ftc.teamcode.lib.Path

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.newSingleThreadContext
import org.firstinspires.ftc.teamcode.Robot.DriveTrain
import org.firstinspires.ftc.teamcode.lib.Constraints.DriveKinematics
import org.firstinspires.ftc.teamcode.lib.Constraints.MotionConstraint
import org.firstinspires.ftc.teamcode.lib.Constraints.PIDFCoefficients
import org.firstinspires.ftc.teamcode.lib.Coords.Point
import org.firstinspires.ftc.teamcode.lib.Coords.Position
import org.firstinspires.ftc.teamcode.lib.Coords.State
import org.firstinspires.ftc.teamcode.lib.Coords.Waypoint
import org.firstinspires.ftc.teamcode.lib.Extensions.*
import kotlin.math.*
import org.firstinspires.ftc.teamcode.lib.Math.Arc
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.system.measureNanoTime
import kotlin.system.measureTimeMillis
import org.firstinspires.ftc.teamcode.OpMode.Robot
import org.firstinspires.ftc.teamcode.Robot.Sensors.IMU


class PathFollower constructor(var path : MutableList<State>, val lookDist : Double, val kinematics: DriveKinematics, val co : PIDFCoefficients, val acceleration : Double, val op : OpMode) {
    var lastClosest : Int = 0
    var previousVelocity : Double = 0.0
    var isDone : Boolean = false
    var reversed : Boolean = false

    fun reset(path: MutableList<State>, direction: Boolean = false) {
        reversed = direction
        DriveTrain.reverseMotors(direction)
        lastClosest = 0
        previousVelocity = 0.0
        isDone = false
        Robot.reset()
        IMU.init(op)
        this.path = path
    }

    fun closestWaypoint(rloc : Position) : State {
        var minDist = Double.MAX_VALUE
        var closestPoint = path.get(0)
        for (i in lastClosest until path.size) {
            val dist = rloc distance path[i]
            if (minDist > dist) {
                minDist = dist
                closestPoint = path[i]
                lastClosest = i
            }
        }
        return closestPoint
    }

    fun followPath(rloc: Position, left : Double, right : Double, time: Double) : List<Double> {
        val closestPoint = closestWaypoint(rloc)
        if (left == 0.0 && right == 0.0 && rloc.x.fuzzyEquals(path.last().x, 1.0) && rloc.y.fuzzyEquals(path.last().y,1.0)) {
            isDone = true
            return listOf(0.0,0.0)
        }
        if (closestPoint == path[path.size-1] || isDone) {
            return listOf(0.0,0.0)
        }
        var looka : Point = closestPoint.next
        for (waypoint in path.subList(lastClosest, path.size)) {
            val lookb = lookAhead(rloc, waypoint)
            if (lookb != null) {
                looka = lookb
                break
            }
        }
        op.telemetry.addData("LookAhead", looka)
        op.telemetry.addData("ClosestPoint", closestPoint)
        val curvature = findCurvature(Position(rloc.x, rloc.y, if(reversed) { rloc.heading + PI} else {rloc.heading}), looka)
        val maxC : Double = time * (acceleration)
        val totalChange : Double = (closestPoint.velocity - previousVelocity).clip(-maxC, maxC)
        val newVelocity : Double = previousVelocity + totalChange
        previousVelocity = newVelocity
        val desiredVelocity = kinematics.getTargetVelocities(min(closestPoint.velocity, newVelocity), curvature)
//        println(desiredVelocity)
        return (PIDF(left,right, desiredVelocity, time))
    }

    fun PIDF(left : Double, right : Double, desiredVelocities : List<Double>, time: Double) : List<Double> {
        val eL = desiredVelocities[0] - left
        val eR = desiredVelocities[1] - right
        val ffl = ((co.kV * desiredVelocities[0]))
        val ffr = ((co.kV * desiredVelocities[1]))
        var leftPower = ffl + eL * co.kP
        var rightPower = ffr + eR * co.kP
        if (leftPower > 1.0 || rightPower > 1.0) {
            if (leftPower > 1.0 && leftPower > rightPower) {
                val mult = 1.0/leftPower
                leftPower *= mult
                rightPower *= mult
            } else if (rightPower > 1.0) {
                val mult = 1.0/rightPower
                leftPower *= mult
                rightPower *= mult
            }
        }
        return listOf(leftPower, rightPower)
    }

    fun lookAhead(rloc: Position, closest : State) : Point? {
        val d = closest.next - closest
        val f = closest - rloc
        val a = d dot d
        val b = 2.0 * (f dot d)
        val c = (f dot f) - lookDist.pow(2.0)
        var discrim = b.pow(2.0) - 4.0 * a * c
        if (discrim < 0) {
            return null
        } else {
            discrim = sqrt(discrim)
            val t0 : Double = (-b - discrim) / (2.0 * a)
            val t1 : Double = (-b + discrim) / (2.0 * a)
            if (t0 in 0.0..1.0) {
                return (closest + (d * t0))
            }
            if (t1 in 0.0..1.0) {
                return (closest + (d * t1))
            }
            //no intersection
        }
        return null
    }

    fun findCurvature(rloc: Position, lookAheadPoint: Point) : Double {
        val a = -tan(rloc.heading)
        val b = 1.0
        val c = tan(rloc.heading) * rloc.x - rloc.y
        val x = abs(a * lookAheadPoint.x + b * lookAheadPoint.y + c) / sqrt(a.pow(2) + b.pow(2))
        val side = sign(sin(rloc.heading) * (lookAheadPoint.x - rloc.x) - cos(rloc.heading) * (lookAheadPoint.y - rloc.y))
        return (2.0 * x / lookDist.pow(2)) * side
    }
}