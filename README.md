# nRF52832 Bluefruit Arduino Libraries

 These's examples for our nRF52832 based Bluefruit LE modules

English | [中文](docs/getting_started_cn.md) | [日本語](docs/getting_started_ja.md)

Welcome to program with Bluefruit52 nRF52832 Core

## 1. Get Started

#### Here is the article to get started

*1.For MacOS*

* [Install the Bluefruit52 board for Arduino](https://www.afantor.cc/nRF52_bluefruit_Learning_Guide.html#arduino-bsp-setup)

*2. For Windows*

* [Install the Bluefruit52 board for Arduino](https://github.com/Afantor/Afantor_Bluefruit52_Arduino/tree/master/docs/getting_started_setting.md) 


## 2. Example

https://github.com/Afantor/Afantor_Bluefruit52_Arduino/tree/master/examples

## 3. API Reference

https://github.com/Afantor/Afantor_Bluefruit52_Arduino/blob/master/src/bluefruit52.h#L49

## 4. H/W Reference

#### Pinout

*We have several kinds of Boards, There is [their difference in schematic](https://github.com/Afantor/Afantor_Bluefruit52_Arduino/tree/master/docs/hardware).*

**LCD**

*LCD Resolution: 135x240*

<table>
 <tr><td>nRF52 Chip</td><td>P0.07</td><td>P0.27</td><td>P0.16</td><td>P0.23</td><td>P0.24</td><td>~CS</td></tr>
 <tr><td>ST7789</td><td>SDA</td><td>SCL</td><td>DC</td><td>RST</td><td>CS</td><td>BL</td></tr>

</table>

**Button**

<table>
 <tr><td>nRF52 Chip</td><td>P0.18</td><td>P0.11</td></tr>
 <tr><td>Button Pin</td><td>BUTTON 1</td><td>BUTTON 2</td></tr>
</table>

**MPU6050**

<table>
 <tr><td>nRF52 Chip</td><td>P0.26</td><td>P0.25</td></tr>
 <tr><td>MPU6050</td><td>SCL</td><td>SDA</td></tr>
</table>


### M-BUS
![image](docs/images/Bluefruit52_Pinconfig.png)
![image](docs/images/Bluefruit52_Pinout.png)

## 5. Awesome Project

* [App-Controller](https://github.com/Afantor/Afantor_Bluefruit52_Arduino/tree/master/examples/Peripheral/controller)  This examples shows you you can use the BLEUart helper class and the Bluefruit LE Connect applications to send based keypad and sensor data to your nRF52.