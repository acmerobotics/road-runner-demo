package com.acmerobotics.roadrunnerdemo;

import com.acmerobotics.roadrunner.path.QuinticSplineSegment;

public class PrintSomeCoefficients {

    public static void main (String[] args) {
        QuinticSplineSegment spline = new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(0,0, 2, 0, 0,0),
                new QuinticSplineSegment.Waypoint(1,2, -2, 2, 0, 0)
        );

        System.out.println(spline.toString());
    }
}
