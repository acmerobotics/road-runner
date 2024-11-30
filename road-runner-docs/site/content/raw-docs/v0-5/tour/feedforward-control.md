---
title: "Feedforward Control"
weight: 3
---

# Feedforward Control

While pure PID control works well for many plants, we can do better. One of the biggest advantages of PID control is its ignorance of the underlying plant. The same three control actions are employed regardless of the relationship between the input and output variables \(as long as its roughly linear\). It's somewhat remarkable that such a simple and general control law is so effective. Nevertheless, we can often leverage knowledge of the plant dynamics to devise improved control laws.

## Gravity Feedforward

For example, consider the problem of controlling the position of a vertical elevator. A simple P controller may help to get close to the desired position. However, close to the setpoint, the elevator will oscillate or stop short with a significant steady-state error due to the effect of gravity. This could be addressed by adding some integral action to the controller. In theory, it will provide the necessary boost to reach the target position. Though as we know, integral terms can easily cause instability and need to be carefully tuned to prevent amplified oscillations. Furthermore, integral action assumes that the resistive force is direction-independent which is not the case here (gravity aids motion downward and hinder motion upward).

The solution is to incorporate the physics of gravity into the control law. As we know from physics, gravity is always exerting a constant downward force with magnitude {{< katex >}} F = mg {{< /katex >}}. To compensate for this, we can add a constant factor {{< katex >}} k_G {{< /katex >}} to the voltage that is sent to the motors.

This can be accomplished using a `PIDFController` with a custom feedforward term:

{{< tabs "gravity controller" >}}
{{< tab "Java" >}}
```java
PIDFController controller = new PIDFController(coeffs, 0, 0, 0, new Function2<Double, Double, Double>() {
    @Override
    public Double invoke(Double position, Double velocity) {
        return kG;
    }
});

// or more concisely with lambdas
PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val controller = new PIDFController(coeffs, kF = { x, v -> kG })
```
{{< /tab >}}
{{</ tabs >}}

This is the essential principle of feedforward: design your control inputs using a model of the plant.

## DC Motor Feedforward

In the realm of wheeled mobile robots, DC motors are important actuators on the drivetrain and other end effectors. To a good approximation, DC motors are linear in normal operating ranges (ignoring friction). Assuming negligible inductance, the voltage necessary for a given velocity and acceleration can be modeled as {{< katex >}} V_{app} = k_v \cdot v + k_a \cdot a {{< /katex >}}. This can be augmented with a term for static friction: {{< katex >}} V_{app} = k_v \cdot v + k_a \cdot a + k_{static} {{< /katex >}}.

{{< hint info >}}
The sign of the static-friction term should match the sign of the sum of the other two terms as the friction will always oppose the motion \(it isn't a purely additive constant\).
{{</ hint >}}

This DC motor feedforward model is directly integrated into `PIDFController` and the `Drive` classes:

{{< tabs "controller update" >}}
{{< tab "Java" >}}
```java
PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic);

controller.setTargetPosition(position);
controller.setTargetVelocity(velocity);
controller.setTargetAcceleration(acceleration);

double correction = controller.update(measuredPosition);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val controller = PIDFController(coeffs, kV, kA, kStatic)

controller.apply {
    targetPosition = position
    targetVelocity = velocity
    targetAcceleration = acceleration
}

val correction = controller.update(measuredPosition)
```
{{< /tab >}}
{{</ tabs >}}

Returning to the first example, it makes more sense to counteract gravity with a constant acceleration feedforward than a custom feedforward as before:

{{< tabs "motor controller update" >}}
{{< tab "Java" >}}
```java
PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic);

controller.setTargetPosition(position);
controller.setTargetVelocity(velocity);
controller.setTargetAcceleration(acceleration + g);

double correction = controller.update(measuredPosition);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val controller = PIDFController(coeffs, kV, kA, kStatic)

controller.apply {
    targetPosition = position
    targetVelocity = velocity
    targetAcceleration = acceleration + g
}

val correction = controller.update(measuredPosition)
```
{{< /tab >}}
{{</ tabs >}}

