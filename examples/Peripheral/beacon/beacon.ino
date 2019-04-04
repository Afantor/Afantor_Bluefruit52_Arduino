/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include <bluefruit52.h>
// Beacon使用广告中的制造商特定数据字段包
// 这意味着您必须提供有效的制造商ID。 
// 更新下面的字段为适当的值。有关有效ID的列表，请参阅:
// https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers

// 0x004C is Apple (示例)
#define MANUFACTURER_ID   0x004C 

// AirLocate UUID: E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
uint8_t beaconUuid[16] = 
{ 
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0, 
};

// 有效的Beacon数据包包含以下信息:
// UUID, Major, Minor, RSSI @ 1M
BLEBeacon beacon(beaconUuid, 0x0000, 0x0000, -54);

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Beacon Example");
  Serial.println("--------------------------\n");

  Bluefruit.begin();

  //关闭蓝色LED以实现最低功耗
  Bluefruit.autoConnLed(false);
  
  //设置最大功率，可选值有：-40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(0);
  Bluefruit.setName("Bluefruit52");

  // 制造商特定数据需要制造商ID
  beacon.setManufacturer(MANUFACTURER_ID);

  // 设置广播包
  startAdv();

  Serial.println("Broadcasting beacon, open your beacon app to test");

  // 暂停循环以节省功耗，因为我们没有任何执行代码
  suspendLoop();
}

void startAdv(void)
{  
  // 广告包
  // 使用填充的BLEBeacon类设置信标有效负载
  // 在此示例的前面
  Bluefruit.Advertising.setBeacon(beacon);

  // 二次扫描响应包（可选）
  // 由于广告包中没有“名称”的空间
  Bluefruit.ScanResponse.addName();
  
  /* 开始做广告
   * - 如果断开连接则启用自动广告
   * - 快速模式超时为30秒
   * - 超时= 0的启动（超时）将永久通告（直到连接）
   * 
   * Apple Beacon规格
   * - 类型：不可连接，无向
   * - 固定间隔： 100 ms -> fast = slow = 100 ms
   */
  //Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);    //以0.625毫秒为单位
  Bluefruit.Advertising.setFastTimeout(30);      // 快速模式下的秒数 
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop() 
{
  // loop is already suspended, CPU will not run loop() at all
}
