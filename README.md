# Line Following with Localization (Webots + Python)

🚀 This project implements a **line-following robot with odometry and localization** in Webots using the e-puck robot.  
The robot follows a black line using ground sensors while simultaneously performing localization with GPS, compass, and LiDAR.

---

## 📌 Features
- ✅ Line following using ground sensors  
- ✅ Odometry and GPS/Compass fusion for localization  
- ✅ LiDAR mapping with visualization on the robot’s display  
- ✅ Coordinate transformation (world → map)  
- ✅ **LiDAR filtering**: infinite readings are replaced with a maximum range value (100m) for better mapping stability   

---

## 🛠️ Requirements
- [Webots](https://cyberbotics.com/) (tested on R2023b)  
- Python 3.8+  
- Libraries:
  ```bash
  pip install numpy matplotlib
