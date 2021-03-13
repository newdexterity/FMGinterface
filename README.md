# A Wearable, Open-Source, Lightweight Forcemyography Armband: On Intuitive, Robust Muscle-Machine Interfaces
## Repository Contents
*	CAD files of the FSR based armband
*	ROS package to interface with the armband

## Force Sensitive Resistors based Armband

With an increasing number of robotic and prosthetic devices, there is a need for intuitive interfaces which enable the user to efficiently interact with them. The conventional interfaces are generally bulky and unsuitable for dynamic and unstructured environments. An alternative to the traditional interfaces is the class of Muscle-Machine Interfaces (MuMIs) that allow the user to have an embodied interaction with the devices they are controlling. In this work, we present a wearable, lightweight, FMG based armband for Human-Machine Interaction fabricated entirely out of 3D-printed parts and silicone components. The armband uses six force sensing units, each housing an FSR sensor. The capabilities of the armband are evaluated while decoding four different gestures (pinch, power, tripod, extension) and rest state and its performance is compared with a state-of-the-art electromyography (EMG) bioamplifier. The decoding performance of the decoding models trained on the data acquired from the armband is significantly better than the performance of the models trained on raw EMG data. The hardware design and the related processing software, are disseminated in an open-source manner.

### Components of the Band

The FMG band can be broken down into several smaller modules; the central control module, the strap module, and sensing modules. These components can be seen in Figure below. In total, there are currently six sensing modules evenly spaced around approximately 320˚ of the strap.

<p align="center">
  <img src = docs/FMG_components.png width="700">
</p>

* The central control module, is responsible for collating all of the data from the sensors, and distributing it to an external device for post processing. An Arduino Nano 33 IoT micro-controller was used to collect the sensor data from the FSRs. This module also allows for the strap to be connected / disconnected, with a form fit keeping the components secured.
* The strap, is made from soft, flexible silicone and is embedded with 3D printed structural components that act as a frame to which the sensor modules are fixed to. This structural component has multiple holes that allow the silicone to cure around the part, ensuring no separation between the  component and the silicone occurs.
* The sensing modules, each contain a single Force Sensitive Resistor (FSR), accompanying circuitry, a transmission block, and a mounting plate for attaching the module to the band.

### Manufacturing Process

Each of the components in this device have been manufactured through Fused Deposition Manufacturing (FDM) processes such as 3D printing, or Hybrid Deposition Manufacturing (HDM).

A hybrid deposition manufacturing process was used to create the strap for the FMG band. A series of steps were employed as follows: First, the mould for the strap was manufactured through 3D printing, along with structural inserts. These components can be seen in the Figure below. A coating of mould release was applied to the mould to aid with removal of the strap. Next, threaded heat inserts were installed in the structural components, and secured to the strap mould with screws. Dragonskin 30 silicone was prepared and de-gassed, then poured into the mould. The mould was allowed to sit to ensure all gasses could escape the mould and allow the silicone to cure around the structural components. Once the silicone was cured, the structural components were unscrewed from the mould, and the completed strap removed.

<p align="center">
  <img src = docs/Manufacturing_process.png width="500">
</p>

The remaining components such as the micro-controller module and mounting components for the sensor modules were 3D printed. Threaded heat inserts were again used for securing the components together.

### Band Placement

The FMG band was placed in the upper half of the forearm where the majority of the muscle groups responsible for the movements of the digits are located. These muscle groups include extensor digitorum, flexor digitorum superficialis, flexor digitorum profundus, and flexor pollicis longus. Figure below shows the placement of the armband.

<p align="center">
  <img src = docs/Band_placement.jpg width="500">
</p>

The performance of the armband was compared with state-of-the-art EMG bioamplifier. To do that, we compare the gesture decoding accuracy for the learning models developed using the data from the FMG armband, raw EMG data, and feature extracted EMG data. A time domain feature, Root Mean Square (RMS) value was extracted for each of the EMG channel. The RMS value represents the square root of the average power of the signal for the given time period. The RMS value is defined as:

<p align="center">
  <img src = docs/Equation.PNG width="200">
</p>

where N is the size of the window applied to the data. Figure below shows the FMG, raw EMG, and feature extracted EMG recordings during the pinch, power, tripod, and extension gestures. Subfigure a) shows the activation values for the FSR sensors, subfigure b) shows the raw EMG activations while subfigure c) shows the feature (RMS value) extracted value from the EMG signals.

<p align="center">
  <img src = docs/Data_plot.png width="700">
</p>

Table below shows the gesture decoding accuracy for each of the three datasets. It should be noted that the accuracy for the gesture decoding models is lowest for the ones developed using the data from raw EMG, while the accuracies for the models developed using the data from FMG and feature extracted EMG are comparable. However, using feature extracted EMG for developing the gesture decoding models introduce an extra step in computation, which might affect the real-time performance of the system.

<p align="center">
  <img src = docs/Table.png width="700">
</p>

To show an application of the proposed FMG armband, the gesture decoding model developed using the data collected for offline training was employed to decode the gestures in real-time and execute them on the NDX-A* robot hand. The demonstrations for the real-time gesture execution on the robot hand were recorded and the compiled video is available in HD quality at the following URL:

[![FMG Band video](docs/FMG_video.png)](https://youtu.be/6V1z5xVdlFU)
