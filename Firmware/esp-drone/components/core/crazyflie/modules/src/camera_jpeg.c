/* Photo-only compression. Native QVGA navigation retains the original Y frame.
 * The low priority worker shares core 0 with the link; camera DMA and flight
 * tasks preempt it. No control commands, NVS writes, or settings are used.
 *
 * The worker's stack is in PSRAM (CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY), as
 * the link task's is (auto_link.c linkTaskStart): it was the one internal-RAM
 * allocation of the photo path that never came back (4096 + 4 B, made at the
 * first photo of a boot and seen as a one-time drop of heapInt / heapFree, which
 * then counts against autonomy's AUTO_HEAP_MIN at every ARM). The condition for
 * a PSRAM stack holds here as it does for the link task: this task never starts
 * a flash operation (no NVS, no settings; fmt2jpg_cb is RAM work, malloc/free and
 * a console print). Its control block stays internal (static). Without PSRAM
 * the stack is internal, as before. */
#include "sdkconfig.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "img_converters.h"
#include "camera_flow.h"
#include "camera_jpeg.h"
#define JPEG_CAPACITY (CAMERA_NATIVE_WIDTH*CAMERA_NATIVE_HEIGHT*2u)
#define JPEG_STACK 4096u /* measured high-water use 2244 B, retaining >1.8 KiB reserve */
static uint32_t state; /* 0 idle; 1 working; 2 done; 3 failed; 4 cancelled, still reading input */
static uint8_t *input, *output;
static uint32_t inputBytes, outputBytes;
static uint8_t inputFormat;
static bool overflow;
static TaskHandle_t worker;
static StaticTask_t workerTcb;
static StackType_t *workerStack;
static size_t append(void *unused, size_t index, const void *data, size_t len) {
  (void)unused;
  if (!data || !len) return 0;
  if (index > JPEG_CAPACITY || len > JPEG_CAPACITY-index) { overflow=true;return 0; }
  memcpy(output+index,data,len);outputBytes=index+len;return len;
}
static void task(void *unused) {
  (void)unused;
  for(;;) {
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    outputBytes=0;overflow=false;
    int64_t started=esp_timer_get_time();
    ESP_LOGI("NATIVEPHOTO","encode start bytes=%u",(unsigned)inputBytes);
    bool ok=fmt2jpg_cb(input,inputBytes,CAMERA_NATIVE_WIDTH,CAMERA_NATIVE_HEIGHT,inputFormat==CAMERA_FLOW_PIXFMT_YUYV?PIXFORMAT_YUV422:PIXFORMAT_GRAYSCALE,
                       85,append,NULL);
    ESP_LOGI("NATIVEPHOTO","encode done ok=%d bytes=%u ms=%u stackFree=%u heap=%u",ok,(unsigned)outputBytes,(unsigned)((esp_timer_get_time()-started)/1000),(unsigned)uxTaskGetStackHighWaterMark(NULL),(unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    uint32_t expected=1;
    uint32_t result=ok && !overflow && outputBytes>4?2:3;
    if(!__atomic_compare_exchange_n(&state,&expected,result,false,__ATOMIC_RELEASE,__ATOMIC_RELAXED))
      __atomic_store_n(&state,0,__ATOMIC_RELEASE); /* cancelled: now safe to reuse the raw slot */
  }
}
bool cameraJpegBusy(void) { return __atomic_load_n(&state,__ATOMIC_ACQUIRE)!=0; }
bool cameraJpegStart(uint8_t *src,uint32_t bytes,uint8_t format) {
  if(cameraJpegBusy() || !src || (format!=CAMERA_FLOW_PIXFMT_YUYV && format!=CAMERA_FLOW_PIXFMT_GRAY8))return false;
  if(!worker) {
    if(!output)output=heap_caps_malloc(JPEG_CAPACITY,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!workerStack) {
#ifdef CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY
      workerStack=heap_caps_malloc(JPEG_STACK,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
#endif
      if(!workerStack)workerStack=heap_caps_malloc(JPEG_STACK,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    }
    if(!output || !workerStack)return false;
    worker=xTaskCreateStaticPinnedToCore(task,"NATIVEPHOTO",JPEG_STACK,NULL,1,workerStack,&workerTcb,0);
    if(!worker)return false;
  }
  input=src;inputBytes=bytes;inputFormat=format;
  __atomic_store_n(&state,1,__ATOMIC_RELEASE);xTaskNotifyGive(worker);return true;
}
int cameraJpegTake(uint8_t *dst,uint32_t capacity,uint32_t *bytes) {
  uint32_t s=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
  if(s==0 || s==1 || s==4)return 0;
  int result=-1;
  if(s==2 && outputBytes<=capacity) { memcpy(dst,output,outputBytes);*bytes=outputBytes;result=1; }
  __atomic_store_n(&state,0,__ATOMIC_RELEASE);return result;
}
void cameraJpegCancel(void) {
  uint32_t s=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
  for(;;) {
    uint32_t next=s==1?4:(s==4?4:0);
    if(__atomic_compare_exchange_n(&state,&s,next,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return;
  }
}
