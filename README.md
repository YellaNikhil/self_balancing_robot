# Self-Balancing Robot

This project showcases the development of a **self-balancing robot** using an Arduino-based platform. By employing a **PID controller** and a **complementary filter approach** for sensor fusion, the robot maintains vertical equilibrium through real-time adjustments to motor commands based on sensor feedback. The project explores concepts in **control theory, robotics, and sensor integration** while emphasizing practical implementation and optimization.

## Key Features
- **Sensor Fusion**: Integrated an **Inertial Measurement Unit (IMU)** combining an accelerometer and gyroscope to achieve precise angle estimation.
- **PID Controller**: Manually tuned PID parameters to ensure optimal performance for maintaining stability at all times.
- **Mechanical Modeling**: Derived a mathematical model of the robot and used **Simulink Simscape** for estimating initial PID coefficients.
- **Hardware Components**:
  - Arduino microcontroller
  - IMU for motion sensing(MPU6050)
  - Two geared DC motors for torque adjustments
  - Custom chassis with precision design

## Project Workflow
1. **Hardware Assembly**:
   - Constructed the robot with an Arduino microcontroller, IMU, and DC motors.
   - Mounted components on a custom-designed steel and acrylic chassis.

2. **Software Development**:
   - Implemented PID control logic for balance adjustment.
   - Developed a complementary filter for sensor fusion to improve angle estimation.

3. **Testing and Optimization**:
   - Iteratively tuned PID parameters through experimentation.
   - Conducted performance tests to verify stability and reliability.

## Results

The two-wheeled self-balancing robot successfully achieved:
- **Stability**: Maintained vertical equilibrium with only angle feedback.
- **Efficient Control**: Demonstrated precise control using the manually tuned PID controller.
- **Robust Performance**: The robot performed reliably under various conditions, making it a practical demonstration of balancing control.

## Recommendations for Future Work

- **Enhanced Sensor Fusion**: Implement Kalman filters for improved angle estimation accuracy.
- **Advanced Control Algorithms**: Explore adaptive or model-predictive controllers for enhanced stability.
- **Real-World Applications**: Integrate additional sensors and features to enable navigation and obstacle avoidance using Ultrasonic Sensor.
