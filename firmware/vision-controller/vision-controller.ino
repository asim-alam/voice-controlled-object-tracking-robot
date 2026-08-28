/*
 * ESP32-CAM Integrated with Voice Command System
 * 
 * Integration with Voice Command ESP32:
 * - Receives bag detection signal from Voice ESP32 Pin 17
 * - Switches from streaming to object detection mode
 * - Sends detection result to Arduino Pin 12
 * - Returns to streaming mode after detection
 * 
 * Pin Connections:
 * - Pin 13: Input from Voice ESP32 Pin 17 (bag detection trigger)
 * - Pin 2: Output to Arduino Pin 12 (object detection result)
 * - Pin 15: Status LED (optional)
 * 
 * Operation Sequence:
 * 1. Default: Streaming mode active
 * 2. Voice ESP32 Pin 17 HIGH -> Switch to object detection
 * 3. Object detected and centered -> Pin 2 HIGH for 500ms
 * 4. Return to streaming mode
 */

#include <Bag_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include "secrets.h"

// WiFi credentials
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Web server on port 80
WebServer server(80);

// GPIO pins for integration
#define VOICE_INPUT_PIN 13    // Input from Voice ESP32 Pin 17
#define ARDUINO_OUTPUT_PIN 2  // Output to Arduino Pin 12
#define STATUS_LED_PIN 15     // Status LED (optional)

// Camera model selection
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif

// Camera constants
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// System variables
static bool debug_nn = false;
static bool is_initialised = false;
static bool streaming_mode = true;
static bool object_detection_active = false;
static bool detection_signal_active = false;
uint8_t *snapshot_buf;

// Timing and control
unsigned long last_voice_trigger = 0;
unsigned long detection_signal_start = 0;
unsigned long last_detection_attempt = 0;
bool last_voice_signal = false;
unsigned long detection_start_time = 0;
// The Arduino scan takes about 54 seconds at 200 ms/degree (center -> right ->
// left), so keep vision active long enough to cover the complete sweep.
const unsigned long DETECTION_TIMEOUT = 65000;
const unsigned long SIGNAL_DURATION = 500;     // 500ms signal to Arduino

// Detection parameters
const float CONFIDENCE_THRESHOLD = 0.7;        // Minimum confidence for detection
// Edge Impulse returns coordinates in model-input space (96x96 for the bundled
// model), not in the raw 320x240 camera frame. Use the center 40% of any model.
const int CENTER_X_MIN = (EI_CLASSIFIER_INPUT_WIDTH * 30) / 100;
const int CENTER_X_MAX = (EI_CLASSIFIER_INPUT_WIDTH * 70) / 100;
const int MIN_OBJECT_WIDTH = (EI_CLASSIFIER_INPUT_WIDTH * 10) / 100;
const int MIN_OBJECT_HEIGHT = (EI_CLASSIFIER_INPUT_HEIGHT * 12) / 100;

// Statistics
unsigned long total_detections = 0;
unsigned long successful_detections = 0;
unsigned long streaming_clients = 0;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ESP32-CAM Integrated Voice-Vision System ===");
    Serial.println("Starting initialization...");

    // Setup GPIO pins
    pinMode(ARDUINO_OUTPUT_PIN, OUTPUT);
    // The voice controller emits an active-HIGH pulse. Keep this line LOW
    // while idle so the same trigger can also feed the drive controller.
    pinMode(VOICE_INPUT_PIN, INPUT_PULLDOWN);
    pinMode(STATUS_LED_PIN, OUTPUT);
    
    digitalWrite(ARDUINO_OUTPUT_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    Serial.println("✅ GPIO pins configured:");
    Serial.printf("   📥 Voice input pin: %d\n", VOICE_INPUT_PIN);
    Serial.printf("   📤 Arduino output pin: %d\n", ARDUINO_OUTPUT_PIN);
    Serial.printf("   💡 Status LED pin: %d\n", STATUS_LED_PIN);

    // Check memory
    Serial.printf("📊 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("📊 Free PSRAM: %d bytes\n", ESP.getFreePsram());

    // Initialize camera
    Serial.println("📷 Initializing camera...");
    if (ei_camera_init() == false) {
        Serial.println("❌ Camera initialization failed!");
        blinkError();
        while(1) delay(1000);
    }
    Serial.println("✅ Camera initialized successfully");

    // Initialize WiFi
    Serial.println("📡 Starting WiFi connection...");
    setupWiFi();
    
    // Initialize web server
    Serial.println("🌐 Starting web server...");
    setupWebServer();

    // Test camera
    testCamera();

    Serial.println("=== SYSTEM READY ===");
    Serial.println("🔄 Streaming mode active");
    Serial.println("🎤 Waiting for voice command trigger");
    Serial.printf("🌐 Web interface: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("📊 Final memory - Heap: %d, PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
    
    // Ready signal
    blinkReady();
}

void loop() {
    yield(); // Watchdog reset
    
    // Handle web server
    server.handleClient();
    
    // Check voice trigger from Voice ESP32
    checkVoiceTrigger();
    
    // Handle detection signal timing
    handleDetectionSignal();
    
    // Run object detection if active
    if (object_detection_active) {
        runObjectDetection();
    }
    
    // Status LED management
    updateStatusLED();
    
    // Periodic status report
    static unsigned long last_status = 0;
    if (millis() - last_status > 10000) {
        printSystemStatus();
        last_status = millis();
    }
    
    delay(10); // Small delay for stability
}

void checkVoiceTrigger() {
    bool voice_signal = digitalRead(VOICE_INPUT_PIN) == HIGH;
    
    // Detect rising edge (voice command received)
    if (voice_signal && !last_voice_signal && streaming_mode) {
        Serial.println("🎤 VOICE TRIGGER DETECTED!");
        Serial.println("📷 Switching to OBJECT DETECTION mode");
        
        // Switch modes
        streaming_mode = false;
        object_detection_active = true;
        detection_start_time = millis();
        last_voice_trigger = millis();
        
        // Visual feedback
        digitalWrite(STATUS_LED_PIN, HIGH);
        
        Serial.println("🔍 Camera now scanning for objects...");
        Serial.println("🎯 Looking for bags in center region");
    }
    
    last_voice_signal = voice_signal;
}

void handleDetectionSignal() {
    // Turn off detection signal after duration
    if (detection_signal_active && (millis() - detection_signal_start) >= SIGNAL_DURATION) {
        digitalWrite(ARDUINO_OUTPUT_PIN, LOW);
        detection_signal_active = false;
        Serial.println("📤 Detection signal to Arduino completed");
        Serial.println("🔄 Returning to STREAMING mode");
        
        // Return to streaming mode
        streaming_mode = true;
        object_detection_active = false;
        Serial.println("✅ Ready for next voice command");
    }
}

void runObjectDetection() {
    // Check timeout
    if (millis() - detection_start_time > DETECTION_TIMEOUT) {
        Serial.println("⏰ Object detection timeout");
        Serial.println("🔄 Returning to streaming mode");
        streaming_mode = true;
        object_detection_active = false;
        return;
    }
    
    // Rate limiting
    if (millis() - last_detection_attempt < 200) {
        return; // Run detection every 200ms
    }
    last_detection_attempt = millis();
    
    // Check memory before allocation
    if (ESP.getFreeHeap() < 100000) {
        Serial.println("⚠️ Low memory, skipping detection");
        return;
    }

    // Allocate snapshot buffer
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if(snapshot_buf == nullptr) {
        Serial.println("❌ Failed to allocate snapshot buffer");
        return;
    }

    // Setup signal for Edge Impulse
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Capture image
    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        Serial.println("❌ Failed to capture image");
        free(snapshot_buf);
        return;
    }

    // Run inference
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    
    if (err != EI_IMPULSE_OK) {
        Serial.printf("❌ Inference failed: %d\n", err);
        free(snapshot_buf);
        return;
    }

    total_detections++;

    // Process results
    bool object_detected_and_centered = false;
    String detection_info = "";

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    Serial.printf("🔍 Inference time: DSP=%dms, Classification=%dms, Anomaly=%dms\n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);
    
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        
        Serial.printf("📦 %s: %.2f%% [x:%u, y:%u, w:%u, h:%u]\n",
                bb.label, bb.value * 100, bb.x, bb.y, bb.width, bb.height);
        
        detection_info += String(bb.label) + ":" + String(bb.value * 100, 1) + "% ";
        
        // Check if object meets detection criteria
        bool confidence_ok = bb.value >= CONFIDENCE_THRESHOLD;
        uint16_t object_center_x = bb.x + (bb.width / 2);
        bool centered_x = (object_center_x >= CENTER_X_MIN && object_center_x <= CENTER_X_MAX);
        bool size_ok = (bb.width >= MIN_OBJECT_WIDTH && bb.height >= MIN_OBJECT_HEIGHT);
        
        Serial.printf("   ✓ Confidence: %s (%.1f%% >= %.1f%%)\n", 
                     confidence_ok ? "PASS" : "FAIL", bb.value * 100, CONFIDENCE_THRESHOLD * 100);
        Serial.printf("   ✓ Centered: %s (center_x=%u, range=%d-%d)\n", 
                     centered_x ? "PASS" : "FAIL", object_center_x, CENTER_X_MIN, CENTER_X_MAX);
        Serial.printf("   ✓ Size: %s (%ux%u >= %dx%d)\n", 
                     size_ok ? "PASS" : "FAIL", bb.width, bb.height, MIN_OBJECT_WIDTH, MIN_OBJECT_HEIGHT);
        
        if (confidence_ok && centered_x && size_ok) {
            object_detected_and_centered = true;
            Serial.println("🎯 PERFECT DETECTION! All criteria met");
            break;
        }
    }
#else
    // Handle classification results
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > CONFIDENCE_THRESHOLD) {
            object_detected_and_centered = true;
            detection_info += String(ei_classifier_inferencing_categories[i]) + ":" + 
                            String(result.classification[i].value * 100, 1) + "% ";
        }
        Serial.printf("📊 %s: %.2f%%\n", ei_classifier_inferencing_categories[i], 
                     result.classification[i].value * 100);
    }
#endif

    // Handle detection result
    if (object_detected_and_centered) {
        Serial.println("🚨 OBJECT DETECTED AND CENTERED! 🚨");
        Serial.printf("📋 Detection: %s\n", detection_info.c_str());
        Serial.println("📤 Sending HIGH signal to Arduino Pin 12");
        
        // Send signal to Arduino
        digitalWrite(ARDUINO_OUTPUT_PIN, HIGH);
        detection_signal_start = millis();
        detection_signal_active = true;
        successful_detections++;
        
        // Visual confirmation
        blinkSuccess();
        
    } else {
        Serial.println("🔍 Object not detected or not centered - continuing scan...");
        if (total_detections % 20 == 0) {
            Serial.printf("📊 Detection attempts: %lu, Success rate: %.1f%%\n", 
                         total_detections, 
                         total_detections > 0 ? (successful_detections * 100.0 / total_detections) : 0);
        }
    }

    free(snapshot_buf);
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    
    Serial.printf("📡 Connecting to: %s\n", ssid);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        if (attempts % 10 == 0) {
            Serial.printf("\n📡 Status: %d, Attempt: %d/30\n", WiFi.status(), attempts);
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected!");
        Serial.printf("🌐 IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("📶 Signal: %d dBm\n", WiFi.RSSI());
    } else {
        Serial.println("\n❌ WiFi connection failed!");
        Serial.println("⚠️ Continuing without WiFi - web interface unavailable");
    }
}

void setupWebServer() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ Skipping web server - no WiFi");
        return;
    }
    
    server.on("/", handleRoot);
    server.on("/stream", handleStream);
    server.on("/capture", handleCapture);
    server.on("/status", handleStatus);
    server.on("/trigger", handleManualTrigger);
    
    server.begin();
    Serial.println("✅ Web server started");
}

void handleRoot() {
    String html = "<!DOCTYPE html>\n"
                  "<html>\n"
                  "<head>\n"
                  "    <title>ESP32-CAM Voice-Vision System</title>\n"
                  "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
                  "    <style>\n"
                  "        body { font-family: Arial; text-align: center; margin: 20px; background: #f0f0f0; }\n"
                  "        .container { max-width: 900px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }\n"
                  "        .status { margin: 20px; padding: 15px; background: #e8f4f8; border-radius: 5px; }\n"
                  "        .detection-zone { border: 3px dashed #ff6b35; position: absolute; left: 31%; top: 25%; width: 38%; height: 50%; }\n"
                  "        .stream-container { position: relative; display: inline-block; }\n"
                  "        img { max-width: 100%; height: auto; border: 1px solid #ccc; }\n"
                  "        .btn { background: #007bff; color: white; border: none; padding: 10px 20px; margin: 5px; border-radius: 5px; cursor: pointer; }\n"
                  "        .btn:hover { background: #0056b3; }\n"
                  "        .btn-danger { background: #dc3545; }\n"
                  "        .btn-danger:hover { background: #c82333; }\n"
                  "        .stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin: 20px 0; }\n"
                  "        .stat-box { background: #f8f9fa; padding: 15px; border-radius: 5px; border: 1px solid #dee2e6; }\n"
                  "    </style>\n"
                  "</head>\n"
                  "<body>\n"
                  "    <div class=\"container\">\n"
                  "        <h1>🤖 ESP32-CAM Voice-Vision System</h1>\n"
                  "        \n"
                  "        <div class=\"status\">\n"
                  "            <h3>System Status</h3>\n"
                  "            <p><strong>Mode:</strong> <span id=\"mode\">" + String(streaming_mode ? "Streaming" : "Object Detection") + "</span></p>\n"
                  "            <p><strong>Voice Trigger:</strong> Pin " + String(VOICE_INPUT_PIN) + " (from Voice ESP32)</p>\n"
                  "            <p><strong>Arduino Signal:</strong> Pin " + String(ARDUINO_OUTPUT_PIN) + " (to Arduino)</p>\n"
                  "            <p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>\n"
                  "        </div>\n"
                  "        \n"
                  "        <div class=\"stream-container\">\n"
                  "            <img id=\"stream\" src=\"/stream\" alt=\"Camera Stream\">\n"
                  "            <div class=\"detection-zone\"></div>\n"
                  "        </div>\n"
                  "        \n"
                  "        <div style=\"margin: 20px 0;\">\n"
                  "            <button class=\"btn\" onclick=\"location.reload()\">🔄 Refresh</button>\n"
                  "            <button class=\"btn\" onclick=\"captureImage()\">📷 Capture</button>\n"
                  "            <button class=\"btn\" onclick=\"checkStatus()\">📊 Status</button>\n"
                  "            <button class=\"btn btn-danger\" onclick=\"manualTrigger()\">🎯 Test Detection</button>\n"
                  "        </div>\n"
                  "        \n"
                  "        <div class=\"stats\">\n"
                  "            <div class=\"stat-box\">\n"
                  "                <h4>📊 Total Detections</h4>\n"
                  "                <p>" + String(total_detections) + "</p>\n"
                  "            </div>\n"
                  "            <div class=\"stat-box\">\n"
                  "                <h4>✅ Successful</h4>\n"
                  "                <p>" + String(successful_detections) + "</p>\n"
                  "            </div>\n"
                  "            <div class=\"stat-box\">\n"
                  "                <h4>📈 Success Rate</h4>\n"
                  "                <p>" + String(total_detections > 0 ? successful_detections * 100 / total_detections : 0) + "%</p>\n"
                  "            </div>\n"
                  "            <div class=\"stat-box\">\n"
                  "                <h4>🧠 Free Memory</h4>\n"
                  "                <p>" + String(ESP.getFreeHeap() / 1024) + " KB</p>\n"
                  "            </div>\n"
                  "        </div>\n"
                  "        \n"
                  "        <div style=\"background: #fff3cd; border: 1px solid #ffeaa7; padding: 15px; border-radius: 5px; margin: 20px 0;\">\n"
                  "            <h4>🔧 Detection Settings</h4>\n"
                  "            <p><strong>Confidence Threshold:</strong> " + String(CONFIDENCE_THRESHOLD * 100, 1) + "%</p>\n"
                  "            <p><strong>Center Region:</strong> X=" + String(CENTER_X_MIN) + "-" + String(CENTER_X_MAX) + " pixels</p>\n"
                  "            <p><strong>Min Object Size:</strong> " + String(MIN_OBJECT_WIDTH) + "x" + String(MIN_OBJECT_HEIGHT) + " pixels</p>\n"
                  "        </div>\n"
                  "    </div>\n"
                  "    \n"
                  "    <script>\n"
                  "        function captureImage() { window.open('/capture', '_blank'); }\n"
                  "        function checkStatus() { window.open('/status', '_blank'); }\n"
                  "        function manualTrigger() {\n"
                  "            if(confirm('Start manual object detection?')) {\n"
                  "                fetch('/trigger').then(() => alert('Detection started!'));\n"
                  "            }\n"
                  "        }\n"
                  "        \n"
                  "        // Auto-refresh mode indicator\n"
                  "        setInterval(function() {\n"
                  "            fetch('/status').then(r => r.text()).then(data => {\n"
                  "                if(data.includes('Object Detection')) {\n"
                  "                    document.getElementById('mode').textContent = 'Object Detection Active';\n"
                  "                    document.getElementById('mode').style.color = 'red';\n"
                  "                } else {\n"
                  "                    document.getElementById('mode').textContent = 'Streaming';\n"
                  "                    document.getElementById('mode').style.color = 'green';\n"
                  "                }\n"
                  "            });\n"
                  "        }, 2000);\n"
                  "    </script>\n"
                  "</body>\n"
                  "</html>";
    
    server.send(200, "text/html", html);
}

void handleStream() {
    WiFiClient client = server.client();
    if (!client) return;
    
    streaming_clients++;
    Serial.println("📺 Client connected to stream");
    
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println("Connection: close");
    client.println();
    
    while (client.connected() && streaming_mode) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            delay(100);
            continue;
        }
        
        if (fb->len > 0) {
            client.printf("--frame\r\n");
            client.printf("Content-Type: image/jpeg\r\n");
            client.printf("Content-Length: %u\r\n\r\n", fb->len);
            client.write(fb->buf, fb->len);
            client.printf("\r\n");
        }
        
        esp_camera_fb_return(fb);
        
        if (!client.connected()) break;
        delay(50); // ~20 FPS
    }
    
    Serial.println("📺 Client disconnected from stream");
}

void handleCapture() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }
    
    server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
}

void handleStatus() {
    String status = "ESP32-CAM Voice-Vision System Status\n\n";
    status += "Mode: " + String(streaming_mode ? "Streaming" : "Object Detection") + "\n";
    status += "Detection Active: " + String(object_detection_active ? "Yes" : "No") + "\n";
    status += "Signal Active: " + String(detection_signal_active ? "Yes" : "No") + "\n";
    status += "Total Detections: " + String(total_detections) + "\n";
    status += "Successful Detections: " + String(successful_detections) + "\n";
    status += "Success Rate: " + String(total_detections > 0 ? successful_detections * 100 / total_detections : 0) + "%\n";
    status += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
    status += "Free PSRAM: " + String(ESP.getFreePsram()) + " bytes\n";
    status += "WiFi Signal: " + String(WiFi.RSSI()) + " dBm\n";
    status += "Uptime: " + String(millis() / 1000) + " seconds\n";
    
    server.send(200, "text/plain", status);
}

void handleManualTrigger() {
    if (streaming_mode) {
        Serial.println("🔧 Manual detection trigger activated");
        streaming_mode = false;
        object_detection_active = true;
        detection_start_time = millis();
        server.send(200, "text/plain", "Manual detection started");
    } else {
        server.send(200, "text/plain", "Detection already active");
    }
}

void testCamera() {
    Serial.println("📷 Testing camera capture...");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("❌ Test capture failed");
        return;
    }
    
    Serial.printf("✅ Test capture success: %d bytes\n", fb->len);
    esp_camera_fb_return(fb);
}

void updateStatusLED() {
    static unsigned long last_blink = 0;
    static bool led_state = false;
    
    if (object_detection_active) {
        // Fast blink during detection
        if (millis() - last_blink > 200) {
            led_state = !led_state;
            digitalWrite(STATUS_LED_PIN, led_state);
            last_blink = millis();
        }
    } else if (streaming_mode) {
        // Slow blink during streaming
        if (millis() - last_blink > 1000) {
            led_state = !led_state;
            digitalWrite(STATUS_LED_PIN, led_state);
            last_blink = millis();
        }
    }
}

void printSystemStatus() {
    Serial.println("=== SYSTEM STATUS ===");
    Serial.printf("🔄 Mode: %s\n", streaming_mode ? "Streaming" : "Object Detection");
    Serial.printf("🔍 Detection: %s\n", object_detection_active ? "Active" : "Inactive");
    Serial.printf("📤 Signal: %s\n", detection_signal_active ? "Active" : "Inactive");
    Serial.printf("📊 Detections: %lu total, %lu successful (%.1f%%)\n", 
                 total_detections, successful_detections, 
                 total_detections > 0 ? (successful_detections * 100.0 / total_detections) : 0);
    Serial.printf("🧠 Memory: %d heap, %d PSRAM\n", ESP.getFreeHeap(), ESP.getFreePsram());
    Serial.printf("📶 WiFi: %d dBm\n", WiFi.RSSI());
    Serial.printf("⏱️ Uptime: %lu seconds\n", millis() / 1000);
    Serial.println("=====================");
}

void blinkError() {
    for (int i = 0; i < 10; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
}

void blinkReady() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(300);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(300);
    }
}

void blinkSuccess() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(50);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(50);
    }
}

// Camera initialization function
bool ei_camera_init(void) {
    if (is_initialised) return true;

    Serial.println("📷 Configuring camera...");
    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera init failed: 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

// Camera capture function for Edge Impulse
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        Serial.println("❌ Camera not initialized");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("❌ Camera capture failed");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if(!converted){
        Serial.println("❌ Format conversion failed");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf, img_width, img_height);
    }

    return true;
}

// Data callback for Edge Impulse
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
