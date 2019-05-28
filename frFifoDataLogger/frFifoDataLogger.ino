// Data logger based on a FIFO to decouple SD write latency from data
// acquisition.

// The FIFO uses two semaphores to synchronize between tasks.

#include <SPI.h>
#include <STM32FreeRTOS.h>
//#include <FreeRTOSConfig.h>
#include <STM32SD.h>

#define SETUP_FLAG      B10000000
#define SCAN_MODE_NONE  B00000110
#define SCAN_MODE_0_N   B00000010
#define CH0TO1          B00001000
#define SCAN_MODE_4_4   B00000100
#define CS_PIN_1        41
#define CS_PIN_2        3
#define MISO_PIN        PA6
#define MOSI_PIN        PA7
#define SCK_PIN         PA5

#define AVGREG          B00100000
#define AVGON           (1 << 4)
#define NAVG4           (0x00 << 2)


#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif


// interval between points in units of 1000 usec
const uint16_t intervalTicks = 1;
//------------------------------------------------------------------------------
// SD file definitions
const uint8_t sdChipSelect = SS;
File file;
//------------------------------------------------------------------------------
// Fifo definitions

// size of fifo in records
const size_t FIFO_SIZE = 200;

// count of data records in fifo
SemaphoreHandle_t fifoData;

// count of free buffers in fifo
SemaphoreHandle_t fifoSpace;

// data type for fifo item plus kalman space
struct FifoItem_t {
  uint32_t usec;
  uint32_t value1;
  uint32_t value2;
  int error;
  double predict_mean1;
  double predict_var1;
  double update_mean1;
  double update_var1;
  double prev1;
  double predict_mean2;
  double predict_var2;
  double update_mean2;
  double update_var2;
  double prev2;
};

// array of data items
FifoItem_t fifoArray[FIFO_SIZE];
//------------------------------------------------------------------------------
// handle for sensor task
TaskHandle_t sens;

//SPI object
SPIClass spi3;

double gaussian(double mu, double sigma2, double x) {
  return 1 / sqrt(2.*PI * sigma2) * exp(-0.5 * pow(x-mu,2) / sigma2);
}

double * mean_update(double mean1, double var1, double mean2, double var2){
  double updated_mean = (var2*mean1 + var1*mean2)/(var1 + var2);
  double updated_var  = 1 / ( 1 / var1 + 1 / var2 );
  static double update_arr[2] = {updated_mean, updated_var};
  return update_arr;
}

double * mean_predict(double mean1, double var1, double mean2, double var2){
  double predicted_mean = mean1 + mean2;
  double predicted_var = var1 + var2;
  static double predict_arr[2] = {predicted_mean, predicted_var};
  return predict_arr;
}

       //Function for reading from the Breakout Accelerometer Boards
uint32_t readADC(int cs_pin) {
  digitalWrite(PC5, LOW);
  delayMicroseconds(1);
  digitalWrite(PC5,HIGH);
  delayMicroseconds(1);
  digitalWrite(PC4, LOW);

  uint32_t data;
  // Send over given modes for our purposes (scan between channel 0 and 1)
  uint16_t regData = SETUP_FLAG | 1 << 3 | SCAN_MODE_0_N | CH0TO1;

  uint16_t msb = spi3.transfer(2, regData);
  uint16_t lsb = spi3.transfer(2, 0x00);

  data = (msb << 6 | lsb >> 2);
  Serial.print(msb << 6);
  Serial.print(",");
  Serial.println(lsb >> 2);
  data <<= 16;

  msb = spi3.transfer(2, 0x00);
  lsb = spi3.transfer(2, 0X00);

  //rearrange the data to fit
  data |= (msb << 6 | lsb >> 2);
  digitalWrite(PC4, HIGH);
  return data;
}

static void Task1(void *arg) {
  // index of record to be filled
  size_t fifoHead = 0;

  // count of overrun errors
  int error = 0;

  // dummy data
  int count = 0;

  // initialise the ticks variable with the current time.
  TickType_t ticks = xTaskGetTickCount();
  double prev1 = 1.;
  double prev2 = 1.;
  double motion1 = 0.;
  double motion2 = 0.;
  double measure_sig = 1000;
  double motion_sig = 60;
  FifoItem_t* p = &fifoArray[fifoHead];
  p->update_var1 = 1000.;
  p->update_mean1 = 0.;
  p->update_var2 = 1000.;
  p->update_mean2 = 0.;
  while (1) {
    // wait until time for next data point
  //  digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelayUntil(&ticks, intervalTicks);

    // get a buffer
    if (xSemaphoreTake(fifoSpace, 0) != pdTRUE) {
      // fifo full - indicate missed point
      error++;
      continue;
    }
    FifoItem_t* p = &fifoArray[fifoHead];
    p->usec = micros();

    // replace next line with data read from sensor
    // f
    p->prev1 = p->value1;
    p->prev2 = p->value2;
    uint32_t readvalue = readADC(PC4);
    p->value1 = (readvalue >> 16) & 0x03FF;
    p->value2 = readvalue & 0x03FF;
//    p->value = count++;
//    motion1 = prev1 - p->value1;
//    motion2 = prev2 - p->value2;
//    
//    double * update_arr1 = mean_update(p->update_mean1, p->update_var1, p->value1, measure_sig);
//    double * predict_arr1 = mean_predict(p->update_mean1, p->update_var1, motion1, motion_sig);
//    double * update_arr2 = mean_update(p->update_mean2, p->update_var2, p->value2, measure_sig);
//    double * predict_arr2 = mean_predict(p->update_mean2, p->update_var2, motion2, motion_sig);
////
//    p->update_mean1 = update_arr1[0];
//    p->update_var1  = update_arr1[1];
////
//    p->predict_mean1 = predict_arr1[0];
//    p->predict_var1 = predict_arr1[1];
//
//    p->update_mean2 = update_arr2[0];
//    p->update_var2  = update_arr2[1];
////
//    p->predict_mean2 = predict_arr2[0];
//    p->predict_var2 = predict_arr2[1];

    p->error = error;
    error = 0;

    // signal new data
    xSemaphoreGive(fifoData);

    // advance FIFO index
    fifoHead = fifoHead < (FIFO_SIZE - 1) ? fifoHead + 1 : 0;
  }
}
//------------------------------------------------------------------------------
// SD write task
static void Task2(void *arg) {
  // FIFO index for record to be written
  size_t fifoTail = 0;

  // time in micros of last point
  uint32_t last = 0;

  while (1) {
    digitalWrite(LED_BUILTIN, LOW);
    // wait for next data record
    xSemaphoreTake(fifoData, portMAX_DELAY);

    FifoItem_t* p = &fifoArray[fifoTail];
    String dataString = "";
    // print interval between points
    if (last) {
      dataString += String(p->usec - last);
      // file.print(F(String(p->usec - last)));
    } else {
      dataString += "NA";
      //      file.print("NA");
    }
//    Serial.print(p->value1);
//    Serial.print(",");
//    Serial.println(p->value2);
    last = p->usec;
    dataString += ",";
    dataString += String(p->value1, DEC);
    dataString += ",";
    dataString += String(p->value2, DEC);
    dataString += ",";
    dataString += String(p->error);
//    dataString += ",";
//    dataString += String(p->update_mean1);
//    dataString += ",";
//    dataString += String(p->update_var1);
//    dataString += ",";
//    dataString += String(p->predict_mean1);
//    dataString += ",";
//    dataString += String(p->predict_var1);
//    dataString += ",";
//    dataString += String(p->update_mean2);
//    dataString += ",";
//    dataString += String(p->update_var2);
//    dataString += ",";
//    dataString += String(p->predict_mean2);
//    dataString += ",";
//    dataString += String(p->predict_var2);
   // dataString += ",";
    //dataString += String(mu);
   // dataString += ",";
  //  dataString += String(sig);
    file.println(dataString);
    //    file.print(",");
    //    file.print((String(p->value)));
    //    file.print(",");
    //    file.println((String(p->error)));
    //    file.println();

    // release record
    xSemaphoreGive(fifoSpace);

    // advance FIFO index
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;

    // check for end run
    if (Serial.available()) {
      // close file to insure data is saved correctly
      file.close();

      // print messages
      Serial.println(F("Done"));
      Serial.print(F("Task1 unused stack entries: "));
      Serial.println(uxTaskGetStackHighWaterMark(sens));
      Serial.print(F("Task2 unused stack entries: "));
      Serial.println(uxTaskGetStackHighWaterMark(0));
      //     ARM free heap not implemented yet
      //     Serial.print(F("Free heap (bytes): "));
      //     Serial.println(freeHeap());
      while (1);
    }
  }
}
//------------------------------------------------------------------------------
void setup() {
  // task creation status
  portBASE_TYPE s1, s2;
//  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PC4, OUTPUT);
  pinMode(PC5, OUTPUT);
  digitalWrite(PC4, HIGH);
  digitalWrite(PC5, HIGH);
  Serial.begin(19200);
  while (!Serial);
  Serial.println(F("Type any character to begin"));
  while (!Serial.available());

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  while (SD.begin(SD_DETECT_PIN) != TRUE)
  {
    delay(10);
  }
  delay(100);
  Serial.println("card initialized.");

  spi3.setMOSI(PA7);
  spi3.setMISO(PA6);
  spi3.setSCLK(PA5);
  spi3 = SPIClass(PA7, PA6, PA5);
  spi3.begin(2);
  file = SD.open("DATA.CSV", FILE_WRITE);
  if (!file) {
    Serial.println("error opening DATA.CSV");
    while (1);
  }
  String header = "time,value1,value2,error);//,update_mean1,update_var1,predict_mean1,predict_var1,update_mean2,update_var2,predict_mean2,predict_var2";
  file.println(header);

  //setup accel
  uint16_t regData = AVGREG | AVGON | NAVG4;
  uint16_t returnData = spi3.transfer(2, regData);
  
  // initialize fifoData semaphore to no data available
  fifoData = xSemaphoreCreateCounting(FIFO_SIZE, 0);

  // initialize fifoSpace semaphore to FIFO_SIZE free records
  fifoSpace = xSemaphoreCreateCounting(FIFO_SIZE, FIFO_SIZE);

  // create sensor task at priority two
  s1 = xTaskCreate(Task1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &sens);

  // create SD write task at priority one
  s2 = xTaskCreate(Task2, NULL, configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL);

  // check for creation errors
  if (fifoData == NULL || fifoSpace == NULL || s1 != pdPASS || s2 != pdPASS ) {
    Serial.println(F("Creation problem"));
    while (1);
  }
  // throw away serial input
  while (Serial.available()) Serial.read();
  Serial.println(F("Type any character to end"));

  // start scheduler
  vTaskStartScheduler();
  Serial.println(F("Insufficient RAM"));
  while (1);
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
  // not used
}
