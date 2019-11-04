package org.firstinspires.ftc.teamcode.lib.Path

import org.firstinspires.ftc.teamcode.lib.Constraints.MotionConstraint
import org.firstinspires.ftc.teamcode.lib.Coords.State

object Paths {
    @JvmStatic
    val straightLine = PathGenerator.generate(mutableListOf(State(0.0,0.0), State(26.0, 0.0)), 2.0, .9,.1,.5, MotionConstraint(30.0, 7.0, 1.0))

    //Use look dist 10
    @JvmStatic
    val straightLineLeft = PathGenerator.generate(mutableListOf(State(0.0,0.0), State(25.5, 4.0)), 2.0, .9,.1,.5, MotionConstraint(30.0, 7.0, 1.0))

    //use look dist 10
    @JvmStatic
    val straightLineRight = PathGenerator.generate(mutableListOf(State(0.0,0.0), State(27.0, -4.0)), 2.0, .9,.1,.5, MotionConstraint(30.0, 7.0, 1.0))

    @JvmStatic
    val backUpStraight = PathGenerator.generate(mutableListOf(State(0.0,0.0), State(-10.0, 0.0)), 2.0, .9,.1,.5, MotionConstraint(30.0, 7.0, 1.0))

    @JvmStatic
    val goToFoundation = PathGenerator.generate(mutableListOf(State(0.0,0.0), State(70.0, 0.0)), 2.0, .9,.1,.5, MotionConstraint(40.0, 10.0, 1.0))

    @JvmStatic
    val pogpursuit = PathGenerator.generate(mutableListOf(State(0.0,0.0),State(58.0,0.0), State(67.0,0.0), State(67.0,-5.0), State(67.0,-5.5), State (67.0,-5.75)), 5.0,.9,.1,1.0,MotionConstraint(40.0,10.0,1.0))
}