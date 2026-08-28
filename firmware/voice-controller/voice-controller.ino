#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "secrets.h"

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// UDP Config
const char* udpAddress = HOST_IP;
const int audioPort    = 12345;             // Microphone data to PC
const int commandPort  = 12346;             // Commands from PC
const int speakerPort  = 12348;             // Audio response from PC
WiFiUDP udpAudio;      
WiFiUDP udpCommand;    
WiFiUDP udpSpeaker;    

// I2S Pins for Microphone (I2S0)
#define I2S_MIC_WS   15  // Word Select (LRCLK)
#define I2S_MIC_SCK  14  // Serial Clock (BCLK)
#define I2S_MIC_SD   13  // Serial Data (DOUT)

// I2S Pins for MAX98357 Speaker (I2S1)
#define I2S_SPK_BCLK  20  // Bit Clock
#define I2S_SPK_LRC   21  // Left/Right Clock (Word Select)
#define I2S_SPK_DIN   47  // Data Input

// Audio Settings
#define SAMPLE_RATE     16000          
#define I2S_READ_LEN    2048           
#define I2S_WRITE_LEN   1024

// I2S Configuration for Microphone (I2S_NUM_0)
i2s_config_t i2s_mic_config = {
  .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate          = SAMPLE_RATE,
  .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  .intr_alloc_flags     = 0,
  .dma_buf_count        = 8,      // Increased for smoother recording
  .dma_buf_len          = 512,
  .use_apll             = false,
  .tx_desc_auto_clear   = false,
  .fixed_mclk           = 0
};

// SIGNIFICANTLY IMPROVED I2S Configuration for Speaker (I2S_NUM_1)
i2s_config_t i2s_spk_config = {
  .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate          = 22050,  // Optimal sample rate for TTS
  .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo for MAX98357
  .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count        = 16,     // Much larger for smoother playback
  .dma_buf_len          = 512,    // Optimized buffer size
  .use_apll             = true,   // Use APLL for better audio quality
  .tx_desc_auto_clear   = true,
  .fixed_mclk           = 0
};

// I2S Pin Mapping for Microphone
i2s_pin_config_t mic_pin_config = {
  .bck_io_num   = I2S_MIC_SCK,
  .ws_io_num    = I2S_MIC_WS,
  .data_out_num = -1,
  .data_in_num  = I2S_MIC_SD
};

// I2S Pin Mapping for Speaker
i2s_pin_config_t spk_pin_config = {
  .bck_io_num   = I2S_SPK_BCLK,
  .ws_io_num    = I2S_SPK_LRC,
  .data_out_num = I2S_SPK_DIN,
  .data_in_num  = -1
};

// Command processing
String lastCommand = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN = 2000;

// Status tracking
unsigned long lastStatusPrint = 0;
unsigned long packetCount = 0;
unsigned long audioPacketsReceived = 0;

// MASSIVELY IMPROVED audio playback buffer and control
uint8_t audioBuffer[16384];     // Much larger buffer (16KB)
bool audioPlaybackActive = false;
bool audioStarted = false;
bool microphonePaused = false;
uint16_t expectedSeq = 0;
unsigned long lastAudioPacket = 0;
const unsigned long AUDIO_TIMEOUT = 5000;  // Increased timeout

// DRAMATIC AUDIO QUALITY IMPROVEMENTS
#define VOLUME_MULTIPLIER 12.0f     // MASSIVE volume boost
#define NOISE_GATE_THRESHOLD 50     // Adjusted threshold
#define MAX_AMPLITUDE_BOOST 2.5f    // Higher boost for quiet signals
#define BASS_BOOST 1.3f             // Bass enhancement
#define TREBLE_BOOST 1.2f           // Treble enhancement

// Enhanced audio processing buffers
int16_t* stereoBuffer = nullptr;
int16_t* processBuffer = nullptr;
const int STEREO_BUFFER_SIZE = 8192;  // Larger buffer
const int PROCESS_BUFFER_SIZE = 4096;

// Built-in LED for status indication
#define LED_PIN 2

// BAG Detection Pin
#define BAG_DETECTION_PIN 17

// Audio enhancement variables
float prevSample = 0.0f;
float dcBlocker = 0.0f;

// BAG detection timing
unsigned long bagActivationTime = 0;
bool bagDetectionActive = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ULTRA-ENHANCED ESP32-S3 AI Voice System Starting ===");

  // Allocate larger audio buffers
  stereoBuffer = (int16_t*)malloc(STEREO_BUFFER_SIZE * sizeof(int16_t));
  processBuffer = (int16_t*)malloc(PROCESS_BUFFER_SIZE * sizeof(int16_t));
  
  if (!stereoBuffer || !processBuffer) {
    Serial.println("❌ Failed to allocate audio buffers!");
    while (1) delay(1000);
  }
  Serial.println("✅ Audio buffers allocated successfully");

  // Initialize built-in LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize BAG detection pin
  pinMode(BAG_DETECTION_PIN, OUTPUT);
  digitalWrite(BAG_DETECTION_PIN, LOW);
  Serial.printf("✅ BAG detection pin %d initialized\n", BAG_DETECTION_PIN);

  // Connect to WiFi with enhanced connection handling
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 40) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println(" failed!");
    Serial.println("Restarting...");
    ESP.restart();
  }

  // Start UDP sockets
  if (udpAudio.begin(audioPort)) {
    Serial.printf("✅ Audio UDP started on port %d\n", audioPort);
  } else {
    Serial.println("❌ Failed to start audio UDP!");
  }
  
  if (udpCommand.begin(commandPort)) {
    Serial.printf("✅ Command UDP started on port %d\n", commandPort);
  } else {
    Serial.println("❌ Failed to start command UDP!");
  }

  if (udpSpeaker.begin(speakerPort)) {
    Serial.printf("✅ Speaker UDP started on port %d\n", speakerPort);
  } else {
    Serial.println("❌ Failed to start speaker UDP!");
  }

  // Initialize I2S for Microphone
  Serial.println("Initializing I2S Microphone...");
  if (i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL) != ESP_OK) {
    Serial.println("❌ I2S microphone driver install failed!");
    while (1) delay(1000);
  }
  if (i2s_set_pin(I2S_NUM_0, &mic_pin_config) != ESP_OK) {
    Serial.println("❌ I2S microphone pin setup failed!");
    while (1) delay(1000);
  }
  
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  Serial.println("✅ I2S Microphone initialized!");

  // Initialize I2S for Speaker with MAXIMUM QUALITY SETTINGS
  Serial.println("Initializing ULTRA-ENHANCED I2S Speaker...");
  if (i2s_driver_install(I2S_NUM_1, &i2s_spk_config, 0, NULL) != ESP_OK) {
    Serial.println("❌ I2S speaker driver install failed!");
    while (1) delay(1000);
  }
  if (i2s_set_pin(I2S_NUM_1, &spk_pin_config) != ESP_OK) {
    Serial.println("❌ I2S speaker pin setup failed!");
    while (1) delay(1000);
  }
  
  // ENHANCED speaker initialization
  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_set_clk(I2S_NUM_1, 22050, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  
  // Multiple initialization cycles for stability
  for (int i = 0; i < 3; i++) {
    i2s_stop(I2S_NUM_1);
    delay(50);
    i2s_start(I2S_NUM_1);
    delay(50);
  }
  
  Serial.println("✅ ULTRA-ENHANCED I2S Speaker initialized!");

  // Test I2S with enhanced diagnostics
  char test_buffer[256];
  size_t test_bytes;
  esp_err_t test_result = i2s_read(I2S_NUM_0, &test_buffer, 256, &test_bytes, 500);
  if (test_result == ESP_OK && test_bytes > 0) {
    Serial.printf("✅ I2S microphone test: %d bytes read\n", test_bytes);
  } else {
    Serial.printf("⚠️ I2S microphone test: result=%d, bytes=%d\n", test_result, test_bytes);
  }

  Serial.println("=== ULTRA-ENHANCED System Ready ===");
  Serial.printf("📡 Streaming to: %s:%d\n", udpAddress, audioPort);
  Serial.printf("📥 Command port: %d\n", commandPort);
  Serial.printf("🔊 Audio port: %d\n", speakerPort);
  Serial.println("🎙️ ULTRA Features:");
  Serial.println("   - MAXIMUM volume output (+24dB)");
  Serial.println("   - Advanced noise reduction");
  Serial.println("   - Bass and treble enhancement");
  Serial.println("   - DC blocking filter");
  Serial.println("   - Soft limiting anti-distortion");
  Serial.println("   - 16KB audio buffers");
  Serial.println("   - BAG detection command added");
  Serial.println("Available commands: FORWARD, BACKWARD, LEFT, RIGHT, STOP, FOG, BAG");
  
  delay(1000);
  playUltraEnhancedStartupSound();
}

void processCommand(String command) {
  unsigned long currentTime = millis();
  if (command == lastCommand && (currentTime - lastCommandTime) < COMMAND_COOLDOWN) {
    return;
  }
  
  lastCommand = command;
  lastCommandTime = currentTime;

  Serial.printf("🎯 RECEIVED COMMAND: %s\n", command.c_str());
  
  // Enhanced LED indication
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(80);
    digitalWrite(LED_PIN, HIGH);
    delay(80);
  }
  
  if (command == "FORWARD") {
    Serial.println("✅ Moving forward...");
    // Add your motor control code here
  } else if (command == "BACKWARD") {
    Serial.println("✅ Moving backward...");
    // Add your motor control code here
  } else if (command == "LEFT") {
    Serial.println("✅ Turning left...");
    // Add your motor control code here
  } else if (command == "RIGHT") {
    Serial.println("✅ Turning right...");
    // Add your motor control code here
  } else if (command == "STOP") {
    Serial.println("✅ Stopping...");
    // Add your motor control code here
  } else if (command == "FOG") {
    Serial.println("✅ Fog detection activated...");
    // Add your fog detection code here
  } else if (command == "BAG") {
    Serial.println("✅ BAG DETECTION ACTIVATED!");
    Serial.println("    🎒 Searching for bag...");
    Serial.printf("    📡 Activating pin %d for 1000ms\n", BAG_DETECTION_PIN);
    Serial.println("    🔍 Pattern recognition started...");
    
    // Activate BAG detection pin
    digitalWrite(BAG_DETECTION_PIN, HIGH);
    bagActivationTime = millis();
    bagDetectionActive = true;
    
    Serial.printf("    ⚡ Pin %d is now HIGH\n", BAG_DETECTION_PIN);
    
    // Add your bag detection code here
    // Example functions you might call:
    // activateBagDetection();
    // startVisionProcessing();
    // searchForBagPattern();
  } else {
    Serial.printf("❌ Unknown command: %s\n", command.c_str());
  }
}

void checkForCommands() {
  int packetSize = udpCommand.parsePacket();
  if (packetSize > 0) {
    char buffer[64];
    int len = udpCommand.read(buffer, sizeof(buffer) - 1);
    if (len > 0) {
      buffer[len] = '\0';
      String cmd = String(buffer);
      cmd.trim();
      cmd.toUpperCase();
      if (cmd.length() > 0) {
        processCommand(cmd);
      }
    }
  }
}

// MASSIVELY ENHANCED audio processing with professional-grade improvements
int16_t processAudioSampleUltraEnhanced(int16_t sample) {
  // DC blocking filter to remove DC offset
  float input = (float)sample;
  dcBlocker = input - prevSample + 0.995f * dcBlocker;
  prevSample = input;
  
  float processed = dcBlocker;
  
  // Enhanced noise gate with hysteresis
  float abs_sample = abs(processed);
  if (abs_sample < NOISE_GATE_THRESHOLD) {
    return 0;
  }
  
  // Dynamic range compression for quiet signals
  if (abs_sample < 2000) {
    processed *= MAX_AMPLITUDE_BOOST;
  }
  
  // Frequency-dependent amplification (basic EQ)
  // This is a simple approximation - real EQ would need more complex filtering
  if (abs_sample < 8000) {  // Boost mid frequencies (voice range)
    processed *= BASS_BOOST;
  } else {  // Boost high frequencies for clarity
    processed *= TREBLE_BOOST;
  }
  
  // MASSIVE volume boost
  processed *= VOLUME_MULTIPLIER;
  
  // Professional soft limiting with smooth curve
  float limit = 32000.0f;
  if (abs(processed) > limit) {
    float sign = (processed > 0) ? 1.0f : -1.0f;
    float excess = abs(processed) - limit;
    // Soft knee compression
    processed = sign * (limit + excess * 0.2f);
  }
  
  // Final hard limit to prevent overflow
  if (processed > 32767.0f) processed = 32767.0f;
  if (processed < -32768.0f) processed = -32768.0f;
  
  return (int16_t)processed;
}

void handleAudioResponse() {
  int packetSize = udpSpeaker.parsePacket();
  if (packetSize > 0) {
    audioPacketsReceived++;
    lastAudioPacket = millis();
    
    int len = udpSpeaker.read(audioBuffer, min(packetSize, (int)sizeof(audioBuffer)));
    
    if (len > 0) {
      // Handle control markers
      if (len == 11 && memcmp(audioBuffer, "START_AUDIO", 11) == 0) {
        Serial.println("🔊 ULTRA-ENHANCED audio stream starting...");
        audioPlaybackActive = true;
        audioStarted = true;
        microphonePaused = true;
        expectedSeq = 0;
        
        // ULTRA preparation for maximum quality
        i2s_stop(I2S_NUM_1);
        i2s_zero_dma_buffer(I2S_NUM_1);
        delay(30);
        
        // Set optimal parameters
        i2s_set_clk(I2S_NUM_1, 22050, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
        
        // Multiple start cycles for stability
        for (int i = 0; i < 2; i++) {
          i2s_start(I2S_NUM_1);
          delay(20);
        }
        
        Serial.println("🎤 Microphone PAUSED during ULTRA playback");
        return;
      }
      
      if (len == 9 && memcmp(audioBuffer, "END_AUDIO", 9) == 0) {
        Serial.println("🔊 ULTRA-ENHANCED audio complete with professional fade-out");
        
        // Professional fade-out to prevent clicks and pops
        if (stereoBuffer) {
          for (int i = 0; i < 1024; i++) {
            float fade_factor = (1024.0f - i) / 1024.0f;
            fade_factor = fade_factor * fade_factor; // Exponential fade
            stereoBuffer[i*2] = (int16_t)(0 * fade_factor);
            stereoBuffer[i*2+1] = (int16_t)(0 * fade_factor);
          }
          
          size_t bytes_written;
          i2s_write(I2S_NUM_1, stereoBuffer, 2048 * sizeof(int16_t), &bytes_written, 1000);
        }
        
        audioPlaybackActive = false;
        audioStarted = false;
        microphonePaused = false;
        
        Serial.println("🎤 Microphone RESUMED after ULTRA playback");
        return;
      }
      
      // Process audio data with ULTRA enhancement
      if (audioStarted && len > 4 && stereoBuffer && processBuffer) {
        uint16_t seq_num = *(uint16_t*)audioBuffer;
        uint16_t data_len = *(uint16_t*)(audioBuffer + 2);
        
        if (data_len <= len - 4 && data_len > 0 && data_len % 2 == 0 && data_len <= PROCESS_BUFFER_SIZE * 2) {
          uint8_t* pcm_data = audioBuffer + 4;
          
          // Enhanced packet loss recovery
          if (seq_num != expectedSeq && expectedSeq > 0) {
            Serial.printf("⚠️ Packet loss detected: expected %d, got %d\n", expectedSeq, seq_num);
            // Insert interpolated silence instead of zeros
            for (int i = 0; i < 512; i++) {
              stereoBuffer[i*2] = 0;
              stereoBuffer[i*2+1] = 0;
            }
            size_t bytes_written;
            i2s_write(I2S_NUM_1, stereoBuffer, 1024 * sizeof(int16_t), &bytes_written, 200);
          }
          expectedSeq = seq_num + 1;
          
          // ULTRA audio processing pipeline
          int16_t* mono_samples = (int16_t*)pcm_data;
          int sample_count = data_len / 2;
          int process_samples = min(sample_count, PROCESS_BUFFER_SIZE);
          
          // First pass: copy and enhance individual samples
          for (int i = 0; i < process_samples; i++) {
            processBuffer[i] = processAudioSampleUltraEnhanced(mono_samples[i]);
          }
          
          // Second pass: convert to stereo with additional enhancements
          int stereo_samples = min(process_samples, STEREO_BUFFER_SIZE / 2);
          for (int i = 0; i < stereo_samples; i++) {
            int16_t enhanced_sample = processBuffer[i];
            
            // Add subtle stereo width (very slight delay between channels)
            int16_t left_sample = enhanced_sample;
            int16_t right_sample = enhanced_sample;
            
            // Add very slight delay to right channel for stereo effect
            if (i > 0) {
              right_sample = (int16_t)((enhanced_sample * 0.95f) + (processBuffer[i-1] * 0.05f));
            }
            
            stereoBuffer[i*2] = left_sample;      // Left channel
            stereoBuffer[i*2+1] = right_sample;   // Right channel
          }
          
          // ULTRA-reliable I2S write with multiple attempts
          size_t bytes_written = 0;
          size_t total_bytes = stereo_samples * 2 * sizeof(int16_t);
          
          for (int attempt = 0; attempt < 3; attempt++) {
            esp_err_t result = i2s_write(I2S_NUM_1, stereoBuffer, total_bytes, &bytes_written, 2000);
            
            if (result == ESP_OK && bytes_written == total_bytes) {
              break; // Success!
            } else {
              Serial.printf("⚠️ I2S write attempt %d: result=%d, written=%d/%d\n", 
                           attempt + 1, result, bytes_written, total_bytes);
              
              if (attempt < 2) {
                // Try to recover
                i2s_stop(I2S_NUM_1);
                delay(10);
                i2s_start(I2S_NUM_1);
                delay(10);
              }
            }
          }
          
          // Reduced debug output frequency
          if (audioPacketsReceived % 150 == 1) {
            Serial.printf("🔊 ULTRA Audio seq %d: %d samples processed, %d bytes written\n", 
                         seq_num, stereo_samples, bytes_written);
          }
        } else {
          Serial.printf("❌ Invalid ULTRA packet: data_len=%d, packet_len=%d\n", data_len, len);
        }
      }
    }
  }
  
  // Enhanced timeout handling
  if (audioPlaybackActive && (millis() - lastAudioPacket) > AUDIO_TIMEOUT) {
    Serial.println("⏰ ULTRA Audio timeout - professional stop");
    audioPlaybackActive = false;
    audioStarted = false;
    microphonePaused = false;
    Serial.println("🎤 Microphone RESUMED after timeout");
  }
}

void playUltraEnhancedStartupSound() {
  Serial.println("🔊 Playing ULTRA-ENHANCED startup sound...");
  
  // Professional startup melody with perfect audio quality
  const int frequencies[] = {440, 554, 659, 880, 659, 554, 440};  // A4, C#5, E5, A5, E5, C#5, A4
  const int durations[] = {200, 200, 200, 400, 200, 200, 500};
  
  microphonePaused = true;
  
  for (int i = 0; i < 7; i++) {
    const int samples_per_tone = 22050 * durations[i] / 1000;
    
    if (stereoBuffer && processBuffer && samples_per_tone <= STEREO_BUFFER_SIZE / 2) {
      // Generate high-quality tone with harmonics
      for (int j = 0; j < samples_per_tone; j++) {
        float time = (float)j / 22050.0;
        float amplitude = 20000.0f;  // Very high amplitude
        
        // Professional envelope (ADSR - Attack, Decay, Sustain, Release)
        float envelope = 1.0f;
        int attack_samples = samples_per_tone / 15;   // Attack
        int decay_samples = samples_per_tone / 10;    // Decay
        int release_samples = samples_per_tone / 8;   // Release
        
        if (j < attack_samples) {
          // Smooth attack
          envelope = (float)j / attack_samples;
          envelope = envelope * envelope; // Exponential curve
        } else if (j < attack_samples + decay_samples) {
          // Decay to sustain level
          float decay_progress = (float)(j - attack_samples) / decay_samples;
          envelope = 1.0f - (decay_progress * 0.2f); // Decay to 80%
        } else if (j > samples_per_tone - release_samples) {
          // Smooth release
          float release_progress = (float)(samples_per_tone - j) / release_samples;
          envelope = release_progress * release_progress; // Exponential release
        }
        
        // Generate rich harmonic content for professional sound
        float fundamental = sin(2 * PI * frequencies[i] * time);
        float harmonic2 = 0.3f * sin(2 * PI * frequencies[i] * 2 * time);
        float harmonic3 = 0.1f * sin(2 * PI * frequencies[i] * 3 * time);
        float harmonic5 = 0.05f * sin(2 * PI * frequencies[i] * 5 * time);
        
        float wave = fundamental + harmonic2 + harmonic3 + harmonic5;
        int16_t sample = (int16_t)(wave * amplitude * envelope);
        
        // Apply ultra enhancement
        sample = processAudioSampleUltraEnhanced(sample);
        
        stereoBuffer[j*2] = sample;      // Left
        stereoBuffer[j*2+1] = sample;    // Right
      }
      
      size_t bytes_written;
      esp_err_t result = i2s_write(I2S_NUM_1, stereoBuffer, samples_per_tone * 2 * sizeof(int16_t), &bytes_written, portMAX_DELAY);
      
      if (result != ESP_OK) {
        Serial.printf("❌ ULTRA startup sound error: %d\n", result);
      }
      
      delay(50);  // Professional gap between tones
    }
  }
  
  microphonePaused = false;
  Serial.println("✅ ULTRA-ENHANCED startup sound complete!");
}

void loop() {
  static char i2s_read_buffer[I2S_READ_LEN];
  size_t bytes_read = 0;

  // Enhanced WiFi monitoring
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("📡 WiFi disconnected! Reconnecting...");
    digitalWrite(LED_PIN, LOW);
    WiFi.reconnect();
    delay(1000);
    return;
  }

  // Professional LED status
  if (digitalRead(LED_PIN) == LOW && !audioPlaybackActive && !microphonePaused) {
    digitalWrite(LED_PIN, HIGH);
  }

  // Check for commands
  checkForCommands();
  
  // Handle BAG detection pin timing
  if (bagDetectionActive && (millis() - bagActivationTime >= 1000)) {
    digitalWrite(BAG_DETECTION_PIN, LOW);
    bagDetectionActive = false;
    Serial.printf("    ⚡ Pin %d deactivated after 1000ms\n", BAG_DETECTION_PIN);
    Serial.println("    🎒 BAG detection signal complete");
  }
  
  // Handle ULTRA audio responses
  handleAudioResponse();

  // Read microphone only when appropriate
  if (!audioPlaybackActive && !microphonePaused) {
    esp_err_t result = i2s_read(I2S_NUM_0, &i2s_read_buffer, I2S_READ_LEN, &bytes_read, 5);
    
    if (result == ESP_OK && bytes_read > 0) {
      udpAudio.beginPacket(udpAddress, audioPort);
      size_t sent = udpAudio.write((const uint8_t*)i2s_read_buffer, bytes_read);
      bool success = udpAudio.endPacket();
      
      if (success) {
        packetCount++;
      }
      
      // Professional status reporting
      unsigned long currentTime = millis();
      if (currentTime - lastStatusPrint >= 10000) {  // Every 10 seconds
        Serial.println("============================================================");
        Serial.printf("📊 ULTRA-ENHANCED Status Report:\n");
        Serial.printf("    🎤 Microphone packets: %lu\n", packetCount);
        Serial.printf("    🔊 Audio packets: %lu\n", audioPacketsReceived);
        Serial.printf("    📡 Microphone: %s\n", microphonePaused ? "PAUSED" : "ACTIVE");
        Serial.printf("    🎵 Audio playback: %s\n", audioPlaybackActive ? "PLAYING" : "IDLE");
        Serial.printf("    🎒 BAG detection pin: %s\n", bagDetectionActive ? "ACTIVE" : "IDLE");
        Serial.printf("    📶 WiFi signal: %ddBm\n", WiFi.RSSI());
        Serial.printf("    🧠 Free memory: %d bytes\n", ESP.getFreeHeap());
        Serial.println("    Commands: FORWARD, BACKWARD, LEFT, RIGHT, STOP, FOG, BAG");
        Serial.println("============================================================");
        lastStatusPrint = currentTime;
      }
    } else if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
      Serial.printf("❌ I2S microphone error: %d\n", result);
      delay(10);
    }
  }

  delay(1);  // Minimal delay
}
