package com.acmerobotics.roadrunner;

public class Test {
    static {
        new Transform2<>(
            new Rotation2<>(new DoubleNum(1.0), new DoubleNum(0.0)),
//            new Rotation2<>(1.0, 0.0),
            new Vector2<>(new DoubleNum(0.0), new DoubleNum(0.0)));
    }
}
