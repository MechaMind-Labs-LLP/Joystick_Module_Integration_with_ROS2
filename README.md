# 🕹️ Joystick Module Integration with ROS 2

This project demonstrates how to integrate a joystick module with a **ROS 2** environment. It involves reading analog joystick input (X, Y axes, and optional button press) from an Arduino, sending it over **Serial**, and using a **ROS 2 Python node** to read and publish that data to `turtle1/cmd_vel`, allowing joystick control of **Turtlesim**. 🐢

---

## 📦 Prerequisites

* Ubuntu 22.04 (recommended)
* Python 3
* Arduino board (e.g., Uno or Nano)
* Joystick module
* ROS 2 Humble installed

---

## 🛠️ Step 1: Install ROS 2 Humble

```bash
sudo apt update && sudo apt upgrade -y

# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install colcon & dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

---

## 🧱 Step 2: Create a ROS 2 Workspace

```bash
mkdir -p ~/joy_ws/src
cd ~/joy_ws
colcon build
source install/setup.bash
```

---

## 📡 Step 3: Arduino Joystick Code (Send Serial Data)

**Wiring:**

* VRx → A0
* VRy → A1
* SW → D3 (optional button)

**Upload the following code using Arduino IDE:**

```cpp
void setup() {
  Serial.begin(115200);
  pinMode(3, INPUT_PULLUP);  // Button
}

void loop() {
  int xVal = analogRead(A0);
  int yVal = analogRead(A1);
  int btn = digitalRead(3);

  Serial.print(xVal);
  Serial.print(",");
  Serial.print(yVal);
  Serial.print(",");
  Serial.println(btn);

  delay(10);  // Adjust as needed
}
```

---

## 🤖 Step 4: ROS 2 Serial Node (Python)

Inside `joy_ws/src`, create a package:

```bash
cd ~/joy_ws/src
ros2 pkg create joy_controller --build-type ament_python --dependencies rclpy geometry_msgs
```

### 📁 Directory Structure:

```
joystick_serial/
├── joystick_serial
│   └── joystick_node.py
├── package.xml
├── setup.py
└── resource/
    └── joystick_serial
```

### ✏️ `joystick_node.py`

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino ✅")
        except serial.SerialException:
            self.get_logger().error("⚠️ Could not connect to Arduino.")
            exit(1)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.ser.readline().decode().strip()
            if line:
                x_val, y_val, btn = map(int, line.split(','))
                msg = Twist()

                # Map joystick values to velocity
                msg.linear.x = (y_val - 512) / 512.0
                msg.angular.z = -(x_val - 512) / 512.0
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 📄 `package.xml`

Replace with:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>joystick_serial</name>
  <version>0.0.1</version>
  <description>Joystick to ROS 2 Turtlesim control via serial</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
</package>
```

---

### 🛠️ `setup.py`

```python
from setuptools import setup

package_name = 'joystick_serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Joystick module integration with ROS 2 via serial',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joystick_node = joystick_serial.joystick_node:main',
        ],
    },
)
```

---

## 🧪 Step 5: Build and Run

```bash
cd ~/joy_ws
colcon build
source install/setup.bash
```

### 🔌 Connect Arduino:

Check the port:

```bash
ls /dev/ttyUSB*
```

If permission denied:

```bash
sudo usermod -a -G dialout $USER
# Then reboot
```

---

## 🐢 Step 6: Launch Everything

### In **Terminal 1** – Launch turtlesim:

```bash
ros2 run turtlesim turtlesim_node
```

### In **Terminal 2** – Run your joystick node:

```bash
source ~/joy_ws/install/setup.bash
ros2 run joystick_serial joystick_node
```

Move your joystick and watch 🐢 `turtle1` move in real time!

---

## 🧰 Useful Terminal Commands

| Command                                                       | Description                            |
| ------------------------------------------------------------- | -------------------------------------- |
| `ros2 node list`                                              | List all available nodes               |
| `ros2 topic list`                                             | List all available topics              |
| `ros2 topic echo /turtle1/cmd_vel`                            | View velocity commands being published |
| `ros2 run turtlesim turtle_teleop_key`                        | Manual keyboard control                |
| `colcon build --packages-select joystick_serial`              | Build only this package                |
| `ros2 pkg create --build-type ament_python your_package_name` | Create a new Python-based package      |

---

## ✅ Extra Tips

* You can tweak sensitivity by adjusting the mapping logic in `joystick_node.py`.
* Use `serial.tools.list_ports` in Python to auto-detect the port.
* Add a launch file for easy startup automation.

---

## 📸 Demo Screenshot (Optional)

*Add a screenshot or video of Turtlesim moving with joystick input.*

---

## 📃 License

MIT License.

---
