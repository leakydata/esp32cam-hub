// Startup: NVS, camera, WiFi, SNTP, the three HTTP servers, mDNS and the worker tasks.
#include "camnode.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_ota_ops.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "secrets.h"

static const char *TAG = "camnode";
static httpd_handle_t httpd80 = NULL, httpd81 = NULL, httpd82 = NULL;
TaskHandle_t hCapture = NULL, hRecorder = NULL, hMotion = NULL, hTimelapse = NULL;
static EventGroupHandle_t wifiEvents;
#define WIFI_GOT_IP BIT0

static void wifiEventHandler(void *, esp_event_base_t base, int32_t id, void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(wifiEvents, WIFI_GOT_IP);
    esp_wifi_connect();  // keep retrying; the AP may just have rebooted
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    printf("\nConnected, IP " IPSTR "\n", IP2STR(&e->ip_info.ip));
    xEventGroupSetBits(wifiEvents, WIFI_GOT_IP);
  }
}

static void connectWiFi() {
  wifiEvents = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *nif = esp_netif_create_default_wifi_sta();
  esp_netif_set_hostname(nif, hostName);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                      wifiEventHandler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                      wifiEventHandler, NULL, NULL));
  wifi_config_t wc = {};
  snprintf((char *)wc.sta.ssid, sizeof(wc.sta.ssid), "%s", WIFI_SSID);
  snprintf((char *)wc.sta.password, sizeof(wc.sta.password), "%s", WIFI_PASS);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
  setRadio(NULL, values[S_RADIO]);
  if (values[S_RADIO]) printf("Radio: robust mode (802.11b/g, no 11n)\n");

  printf("Connecting to %s", WIFI_SSID);
  uint32_t start = millis();
  while (!(xEventGroupGetBits(wifiEvents) & WIFI_GOT_IP)) {
    delayMs(250);
    printf(".");
    fflush(stdout);
    // An AP with legacy rates disabled refuses an 802.11b-only client outright
    // ("Refused basic rates mismatch"), and S_RADIO is persistent -- without this
    // fallback one toggle would strand the camera until someone reflashed it by USB.
    if (values[S_RADIO] && millis() - start > 15000) {
      printf("\nNo link in robust mode, falling back to b/g/n\n");
      values[S_RADIO] = 0;
      prefsSetInt("radio", 0);
      setRadio(NULL, 0);
      esp_wifi_disconnect();
      delayMs(100);
      esp_wifi_connect();
      start = millis();
      continue;
    }
    if (millis() - start > 30000) {
      printf("\nWiFi connect timeout, restarting\n");
      esp_restart();
    }
  }
}

static httpd_handle_t startHttpd(uint16_t port, uint16_t ctrlPort, int sockets,
                                 const httpd_uri_t *uris, int n) {
  httpd_handle_t h = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port;
  config.ctrl_port = ctrlPort;
  config.max_open_sockets = sockets;
  config.lru_purge_enable = true;
  config.stack_size = 6144;
  config.uri_match_fn = httpd_uri_match_wildcard;
  config.recv_wait_timeout = 20;  // a 1.1MB OTA upload over a weak link stalls at 5s
  config.send_wait_timeout = 20;
  if (httpd_start(&h, &config) != ESP_OK) {
    printf("HTTP server on port %u failed to start\n", port);
    return NULL;
  }
  for (int i = 0; i < n; i++) httpd_register_uri_handler(h, &uris[i]);
  return h;
}

// Close the clip, stop the workers and shut the camera down. Flash erase stops the
// cache; anything still executing from flash or driving DMA through it at that moment
// takes the whole chip down with the interrupt watchdog.
void quiesceForOta() {
  stopRecording();
  if (hTimelapse) vTaskSuspend(hTimelapse);
  if (hMotion) vTaskSuspend(hMotion);
  if (hRecorder) vTaskSuspend(hRecorder);
  if (hCapture) vTaskSuspend(hCapture);
  delayMs(100);
  if (cameraOk) esp_camera_deinit();  // stops the I2S/DMA engine feeding frames
  delayMs(50);
}

void startServers() {
  int nApi, nStream, nFiles;
  const httpd_uri_t *api = apiUris(&nApi);
  const httpd_uri_t *stream = streamUris(&nStream);
  const httpd_uri_t *files = fileUris(&nFiles);
  // lwIP has 16 sockets total; each server also uses a listen + control socket
  httpd80 = startHttpd(80, 32768, 4, api, nApi);
  httpd81 = startHttpd(81, 32769, 2, stream, nStream);
  httpd82 = startHttpd(82, 32770, 2, files, nFiles);
}

static void startMdns() {
  if (mdns_init() != ESP_OK) return;
  mdns_hostname_set(hostName);
  mdns_instance_name_set(hostName);
  mdns_txt_item_t txt[] = {
    {"version", FW_VERSION},
    {"stream_port", "81"},
    {"files_port", "82"},
  };
  mdns_service_add(NULL, "_espcam", "_tcp", 80, txt, 3);
  mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

extern "C" void app_main(void) {
  printf("\ncamnode " FW_VERSION "\n");
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // factory MAC from efuse: a stable name that does not need WiFi to be up first
  uint8_t mac[6] = {};
  esp_efuse_mac_get_default(mac);
  snprintf(hostName, sizeof(hostName), "espcam-%02x%02x%02x", mac[3], mac[4], mac[5]);

  sdLock = xSemaphoreCreateRecursiveMutex();

  // Without a working camera the rest still starts, so the board stays reachable
  // over WiFi (status, OTA) instead of rebooting in a loop.
  if (initCamera()) {
    xTaskCreatePinnedToCore(captureTask, "capture", 4096, NULL, 5, &hCapture, 1);
  } else {
    loadSettings(NULL);  // settings still need to exist for /status and /control
  }

  connectWiFi();

  setenv("TZ", TZ_INFO, 1);
  tzset();
  esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  esp_netif_sntp_init(&sntp);

  startServers();
  xTaskCreatePinnedToCore(recorderTask, "recorder", 8192, NULL, 3, &hRecorder, 0);
  xTaskCreatePinnedToCore(motionTask, "motion", 8192, NULL, 2, &hMotion, 0);
  xTaskCreatePinnedToCore(timelapseTask, "timelapse", 6144, NULL, 2, &hTimelapse, 0);
  startMdns();

  esp_netif_ip_info_t ip = {};
  esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);
  printf("Ready: http://%s.local/  http://" IPSTR "/\n", hostName, IP2STR(&ip.ip));

  // We got WiFi, the servers are listening and the tasks are up: this image works.
  // Until this call the bootloader holds the update pending, and a reset would go back
  // to the previous slot -- so a bad OTA can never leave a camera unreachable.
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
      state == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    printf("Update confirmed, running from %s\n", running->label);
  }
}
