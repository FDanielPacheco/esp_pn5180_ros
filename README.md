# ROS-ESP-PN5180

**ESP-PN5180 ROS-Noetic** is an ROS package developed to interface with the ESP-PN5180 via serial port. The package facilitates data exchange between the RFID reader and ROS-based systems through the definition of custom message types and topic-based communication. It supports structured data handling, specifically for cargo information, and unstructured text-based data exchange. Functionality includes read/write operations on RFID tags, configuration of serial communication parameters, retrieval of device firmware version, and control of operational states. The package provides a modular approach to integrating RFID capabilities into robotic and automation systems utilizing the ROS architecture.

---

## Getting Started

### Installation

To get started, ensure you have ROS 1 Noetic installed.

1. Clone and install the serial port library repository:
```
git clone https://github.com/FDanielPacheco/libserialposix.git
make install/c
```
2. Create a ROS Workspace (if you don't have one) (optional):
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
3. Clone the ESP-PN5180 package repository:
```
git clone <repo>
```
4. Source ROS libraries and environment variables (from the ```catkin_ws``` directory):
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```
5. Compile the workspace:
```
catkin_make 
```
6. Start the nodes responsible for ESP-PN5180:\n
It's necessary to know where is the device ESP-PN5180 connected, to identify the correct port, you can use various tools. Once identified (e.g., /dev/ttyUSB0), replace ttyXXXX with the correct port in the following commands.
```
rosrun esp_pn5180_ros rfid_supervisor_node -p /dev/ttyXXXX 
```

---

## Interface

This ROS node, rfid_supervisor_node, communicates using the following topics:

Published Topics:

1.  **Receiving Data:**

    * **Logs:** 
        * Subscribe to this topic `/rfid/logs` to receive log messages for debugging and monitoring the node's operation.
    * **Version:** 
        * Subscribe to this topic `/rfid/version` to receive the firmware version of the ESP-PN5180 device.
    * **Serial Configuration:** 
        * Subscribe to this topic `/rfid/serial` to receive the serial port configuration.
    * **Serial Output:** 
        * Subscribe to this topic `/rfid/serial/out` to receive all data received from the ESP-PN5180's serial port.
    * **Serial Input:** 
        * Subscribe to this topic `/rfid/serial/in/mon` to monitor all data sent to the ESP-PN5180's serial port.
    * **JSON Tag:** 
        * Subscribe to this topic `/rfid/tag` to receive RFID tag information in JSON format, as received from the serial port.
    * **Cargo tag structure:** 
        * Subscribe to this topic `/rfid/tag/cargo` to receive RFID tag information in the Produtech `tagCargo` protocol message format.
    * **MRTHS tag structure:** 
        * Subscribe to this topic `/rfid/tag/mrths` to receive RFID tag information in the MRTHS `tagMRTHS` protocol message format.

2.  **Sending Requests:**

    * **Cargo Data:**
        * Publish a `esp_pn5180_ros/RequestCargo` message to the appropriate topic (e.g., `/rfid/request/cargo`).
        * Populate the `vehicle`, `content`, and `destination` fields within the `arg` field of the message.
    * **MRTHS Data:**
        * Publish a `esp_pn5180_ros/RequestMrths` message to the appropriate topic (e.g., `/rfid/request/mrths`).
        * Set the `text` field within the `arg` field of the message.
    * **Setting Timeout:**
        * Publish a `esp_pn5180_ros/RequestSetTimeout` message to the appropriate topic (e.g., `/rfid/request/setTimeout`).
        * Set the `state` (true/false) and `time` (milliseconds) fields.
    * **Sending Serial Commands:**
        * Publish a `esp_pn5180_ros/RequestSerial` message to the appropriate topic (e.g., `/rfid/serial/in`).
        * Set the `arg` field to the serial command string.
    * **Requesting Version:**
        * Publish a `esp_pn5180_ros/RequestVersion` message to the correct topic (e.g. `/rfid/version/get`).
    * **Stopping Operations:**
        * Publish a `esp_pn5180_ros/RequestStop` message to the correct topic (e.g. `/rfid/stop`).

---

## Troubleshooting

* **Serial Port Issues:**
    * If you encounter issues with the serial port, double-check the port path and permissions.
    * Ensure the ESP-PN5180 is properly connected.
* **Message Errors:**
    * Verify the message structures and data types when publishing messages.
    * Use `rostopic echo` to inspect the messages being published and received.

---

## License

This project is licensed under the [MIT License](../../LICENSE).

---

## Contact

For any questions or issues, please reach out via email at pacheco.castro.fabio@gmail.com or fabio.d.pacheco@inesctec.pt.
