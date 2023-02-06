# Inertial Measurement Unit Code
---

All code related to IMU may be found within this repository. Initial IMU testing was done with the [Adafruit BNO055 9-DOF Absolute Orientation IMU](https://www.adafruit.com/product/2472). The datasheet for this product can be found at this [link](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf). This IMU interfaced perfectly with the Arduino suite of microcontrollers over I2C, and the examples in this repository will work in this context. However, several set backs were discovered with this model of IMU, thus a new IMU is being used going forward. We are now using the [CMPS14 Tilt Compensated Magnetic Compass](https://ca.robotshop.com/products/tilt-compensated-magnetic-compass-cmps14). The datasheet for this product can be found at this [link](https://cdn.robotshop.com/media/d/dev/rb-dev-98/pdf/tilt-compensated-magnetic-compass-cmps14-datasheet.pdf). This device uses different, more accurate fusion algorithms compared to the BNO055, and allows for the use of a static factory calibration.

Author: Ethan Garnier
